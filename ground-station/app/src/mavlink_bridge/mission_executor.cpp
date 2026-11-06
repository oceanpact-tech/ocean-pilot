/**
 ******************************************************************************
 * @file    mission_executor.cpp
 * @brief   Implementation of the mission interpreter.
 *
 * This file implements the MissionExecutor class. The whole mission runs as 
 * one ActiveManeuver::Mission on the ManeuverRunner thread, and each item is 
 * executed through FlightPrimitives.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/mission_executor.hpp"

#include <chrono>
#include <format>

// Composed modules
#include "mavlink_bridge/flight_action_executor.hpp"
#include "mavlink_bridge/ground_station_logger.hpp"
#include "mavlink_bridge/maneuver_runner.hpp"

// Mavlink types
#include "mavlink_bridge/custom_types/mav_cmd.hpp"
#include "mavlink_bridge/custom_types/mav_severity.hpp"

// Helpers
#include "mavlink_bridge/utils/mission_item_utils.hpp"


/* Aliases -------------------------------------------------------------------*/

using Maneuver = FlightStateManager::ActiveManeuver;
using Scope    = FlightStateManager::ActionScope;
using Policy   = ManeuverRunner::Policy;


/* Local helpers -------------------------------------------------------------*/

namespace
{
    /// @brief Takeoff, land and return to launch are native DJI actions that cannot be interrupted midway.
    bool is_native_action(uint32_t command)
    {
        return command == MavCmd::MAV_CMD_NAV_TAKEOFF ||
               command == MavCmd::MAV_CMD_NAV_LAND    ||
               command == MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH;
    }

    /// @brief Converts a MAVLink duration in seconds to milliseconds (never negative).
    std::chrono::milliseconds to_milliseconds(float seconds)
    {
        return std::chrono::milliseconds(static_cast<int64_t>(std::max(seconds, 0.0f) * 1000.0f));
    }
}


/* Public Class Methods ------------------------------------------------------*/

MissionExecutor::MissionExecutor(
        // DJI API
        DJIInterface& dji,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives&     flight_primitives,
        MissionHandler&       mission_handler,
        GroundStationLogger&  ground_station_logger,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher&   telemetry_publisher,
        ManeuverRunner&       maneuver_runner
)
    : // DJI API
      dji_(dji),
      // Composed modules
      state_manager_(state_manager),
      flight_primitives_(flight_primitives),
      flight_action_executor_(flight_action_executor),
      mission_handler_(mission_handler),
      ground_station_logger_(ground_station_logger),
      manual_control_handler_(manual_control_handler),
      telemetry_publisher_(telemetry_publisher),
      maneuver_runner_(maneuver_runner)
{
}

bool MissionExecutor::start()
{
    // The heartbeat echoes the Mission mode back as a mode-change request: ignore it while running.
    if (this->is_active())
        return false;

    const auto snapshot = this->mission_handler_.get_plan();
    if (!snapshot)
    {
        this->ground_station_logger_.send_message(MavSeverity::WARNING, "Mission refused: no mission uploaded");
        return false;
    }

    if (this->dji_.is_manual_override_active())
    {
        this->ground_station_logger_.send_message(MavSeverity::WARNING, "Mission refused: pilot has control");
        return false;
    }

    // Resume the paused mission, or start from the first item.
    Progress progress{true, snapshot->version, 0, 5.0f};
    bool resuming = false;
    {
        std::lock_guard<std::mutex> lock(this->progress_mutex_);
        if (this->progress_.valid && this->progress_.plan_version == snapshot->version)
        {
            progress = this->progress_;
            resuming = true;
        }
    }

    // On the ground the plan must begin with a takeoff.
    if (!this->dji_.is_armed() && !starts_with_takeoff(snapshot->plan, progress.next_index))
    {
        this->ground_station_logger_.send_message(MavSeverity::WARNING, "Mission refused: add a takeoff item");
        return false;
    }

    this->pause_requested_ = false;

    const bool started = this->maneuver_runner_.run(
        Maneuver::Mission, Policy::RejectIfBusy,
        [this, plan = *snapshot, progress](std::stop_token stoken, Scope& scope)
        {
            this->run_mission(plan, progress, stoken, scope);
        }
    );

    if (!started)
    {
        this->ground_station_logger_.send_message(MavSeverity::WARNING, "Mission refused: drone is busy");
        return false;
    }

    this->ground_station_logger_.send_message(
        MavSeverity::INFO,
        resuming ? std::format("Mission resumed at item {}", progress.next_index) : std::string("Mission started")
    );

    return true;
}

void MissionExecutor::pause()
{
    if (!this->is_active())
        return;

    this->pause_requested_ = true;
    this->maneuver_runner_.cancel(); // Blocks until the mission body unwound (and braked).
}

void MissionExecutor::reset_progress()
{
    std::lock_guard<std::mutex> lock(this->progress_mutex_);
    this->progress_ = Progress{};
}

bool MissionExecutor::is_active() const
{
    return this->state_manager_.get_active_maneuver() == Maneuver::Mission;
}

bool MissionExecutor::is_paused() const
{
    std::lock_guard<std::mutex> lock(this->progress_mutex_);
    return this->progress_.valid && !this->is_active();
}


/* Private Class Methods -----------------------------------------------------*/

void MissionExecutor::run_mission(
    const MissionHandler::PlanSnapshot& snapshot,
    Progress progress,
    std::stop_token stoken,
    Scope& scope
)
{
    const auto& items = snapshot.plan.mission_items;

    for (std::size_t index = progress.next_index; index < items.size(); index++)
    {
        const auto& item = items[index];

        const Outcome outcome = this->run_item(item, progress, stoken, scope);

        if (outcome == Outcome::Done)
        {
            // Reported for every item (even skipped ones) so MISSION_CURRENT stays aligned with the plan.
            this->mission_handler_.report_item_reached();
            continue;
        }

        if (outcome == Outcome::Cancelled)
        {
            if (this->pause_requested_.exchange(false))
            {
                // Pause: hold position and keep the progress.
                if (item.command == MavCmd::MAV_CMD_NAV_WAYPOINT)
                    this->flight_primitives_.brake(scope);

                // A native action keeps running by itself, so the mission resumes after it.
                progress.next_index = is_native_action(item.command) ? index + 1 : index;
                this->save_progress(progress);

                this->ground_station_logger_.send_message(
                    MavSeverity::INFO, 
                    std::format("Mission paused at item {}", progress.next_index)
                );
            }
            else
            {
                // Preempted by land, RTL or shutdown: nothing to resume.
                this->reset_progress();
                this->ground_station_logger_.send_message(
                    MavSeverity::INFO, 
                    "Mission cancelled"
                );
            }
            return;
        }

        if (outcome == Outcome::Overridden)
        {
            this->reset_progress();
            this->ground_station_logger_.send_message(MavSeverity::WARNING, "Mission aborted: pilot took control");
            return;
        }

        // Failed or timed out.
        this->reset_progress();
        this->return_to_launch_after_failure(index, stoken, scope);
        return;
    }

    this->reset_progress();
    this->ground_station_logger_.send_message(MavSeverity::INFO, "Mission complete");
    scope.succeed();
}

Outcome MissionExecutor::run_item(
    const mavsdk::MissionRawServer::MissionItem& item,
    Progress& progress,
    std::stop_token stoken,
    Scope& scope
)
{
    switch (item.command)
    {
        case MavCmd::MAV_CMD_NAV_TAKEOFF:
            // Already airborne (e.g. resumed or started in flight): nothing to do.
            if (this->dji_.is_armed())
                return Outcome::Done;
            return this->flight_primitives_.takeoff(scope, stoken);

        case MavCmd::MAV_CMD_NAV_WAYPOINT:
        {
            const auto command = to_waypoint_command(
                item, static_cast<float>(this->dji_.get_home_altitude()), progress.speed_m_s
            );

            if (!command)
            {
                this->ground_station_logger_.send_message(MavSeverity::ERROR, std::format("Item {}: unsupported frame", item.seq));
                return Outcome::Failed;
            }

            Outcome outcome = this->flight_primitives_.go_to_waypoint(scope, *command, stoken);

            // param1 = hold time at the waypoint.
            if (outcome == Outcome::Done && item.param1 > 0.0f)
                outcome = this->flight_primitives_.hold(to_milliseconds(item.param1), stoken);

            return outcome;
        }

        case MavCmd::MAV_CMD_DO_CHANGE_SPEED:
            if (item.param2 > 0.1f)
                progress.speed_m_s = item.param2;
            return Outcome::Done;

        case MavCmd::MAV_CMD_NAV_DELAY:
        case MavCmd::MAV_CMD_NAV_LOITER_TIME:
            return this->flight_primitives_.hold(to_milliseconds(item.param1), stoken);

        case MavCmd::MAV_CMD_NAV_LAND:
            return this->flight_primitives_.land(scope, stoken);

        case MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH:
            return this->flight_primitives_.return_to_launch(scope, stoken);

        default:
            return Outcome::Done; // Unsupported items were reported at upload and are skipped.
    }
}

void MissionExecutor::return_to_launch_after_failure(
    std::size_t failed_item,
    std::stop_token stoken,
    Scope& scope
)
{
    if (!this->dji_.is_armed())
    {
        this->ground_station_logger_.send_message(MavSeverity::ERROR, std::format("Item {} failed on the ground", failed_item));
        return;
    }

    this->ground_station_logger_.send_message(MavSeverity::ERROR, std::format("Item {} failed: returning home", failed_item));

    // The reported mode becomes ReturnToLaunch inside the same scope.
    if (!scope.transition_to(Maneuver::ReturnToLaunch))
        return;

    const Outcome outcome = this->flight_primitives_.return_to_launch(scope, stoken);

    if (outcome != Outcome::Done && outcome != Outcome::Cancelled)
        this->ground_station_logger_.send_message(MavSeverity::ERROR, std::format("RTL {}", to_string(outcome)));
}

void MissionExecutor::save_progress(const Progress& progress)
{
    std::lock_guard<std::mutex> lock(this->progress_mutex_);
    this->progress_ = progress;
    this->progress_.valid = true;
}