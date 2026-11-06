/**
 ******************************************************************************
 * @file    flight_action_executor.cpp
 * @brief   Implementation of the flight command entry points.
 *
 * This file implements the FlightActionExecutor class. Each command validates 
 * its preconditions, sends the COMMAND_ACK feedback and runs a maneuver body 
 * on the ManeuverRunner thread through FlightPrimitives.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/flight_action_executor.hpp"

#include <cstdint>
#include <format>
#include <optional>

// Composed modules
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/flight_primitives.hpp"
#include "mavlink_bridge/flight_action_executor.hpp"
#include "mavlink_bridge/mission_handler.hpp"
#include "mavlink_bridge/mission_executor.hpp"
#include "mavlink_bridge/ground_station_logger.hpp"
#include "mavlink_bridge/manual_control_handler.hpp"
#include "mavlink_bridge/telemetry_publisher.hpp"
#include "mavlink_bridge/maneuver_runner.hpp"

// Mavlink types
#include "mavlink_bridge/custom_types/mav_cmd.hpp"
#include "mavlink_bridge/custom_types/mav_result.hpp"
#include "mavlink_bridge/custom_types/mav_severity.hpp"

// Helpers
#include "mavlink_bridge/utils/mavlink_json_utils.hpp"


/* Aliases -------------------------------------------------------------------*/

using Maneuver = FlightStateManager::ActiveManeuver;
using Scope    = FlightStateManager::ActionScope;
using Policy   = ManeuverRunner::Policy;


/* Local helpers -------------------------------------------------------------*/

namespace
{
    /// @brief Maps an Outcome to the MAV_RESULT of the final ACK. Empty when no ACK must be sent (cancellation).
    std::optional<uint8_t> ack_for(Outcome outcome)
    {
        switch (outcome)
        {
            case Outcome::Done:       return MavResult::ACCEPTED;
            case Outcome::Cancelled:  return std::nullopt;
            case Outcome::Failed:
            case Outcome::Timeout:
            case Outcome::Overridden: return MavResult::FAILED;
        }
        return MavResult::FAILED;
    }
}


/* Public Class Methods ------------------------------------------------------*/

FlightActionExecutor::FlightActionExecutor(
    // DJI API
    DJIInterface& dji,
    // MAVSDK API
    mavsdk::MavlinkDirectServer& mavlink_direct_server,
    // Composed modules
    FlightStateManager& state_manager,
    FlightPrimitives& flight_primitives,
    MissionHandler& mission_handler,
    MissionExecutor& mission_executor,
    GroundStationLogger& ground_station_logger,
    ManualControlHandler& manual_control_handler,
    TelemetryPublisher& telemetry_publisher,
    ManeuverRunner& maneuver_runner
)
    : // DJI API
      dji_(dji),
      // MAVSDK API
      mavlink_direct_server_(mavlink_direct_server),
      // Composed modules
      state_manager_(state_manager),
      flight_primitives_(flight_primitives),
      mission_handler_(mission_handler),
      mission_executor_(mission_executor),
      ground_station_logger_(ground_station_logger),
      manual_control_handler_(manual_control_handler),
      telemetry_publisher_(telemetry_publisher),
      maneuver_runner_(maneuver_runner)
{
}

void FlightActionExecutor::handle_arm_disarm(const mavsdk::ActionServer::ArmDisarm& command)
{
    // Command = ARM
    if (command.arm)
    {
        if (!this->dji_.is_ready_to_takeoff())
        {
            this->send_command_ack(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 
                MavResult::DENIED
            );
            return;
        }

        this->send_command_ack(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 
            MavResult::ACCEPTED
        );
        return;
    }

    // Command = DISARM. The DJI SDK cannot disarm, so answer instead of letting the GCS time out.
    this->send_command_ack(
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 
        MavResult::DENIED
    );
}

void FlightActionExecutor::execute_takeoff()
{
    const bool started = this->maneuver_runner_.run(
        Maneuver::Takeoff, Policy::RejectIfBusy,
        [this](std::stop_token stoken, Scope& scope)
        {
            if (!this->dji_.is_ready_to_takeoff())
            {
                this->send_command_ack(MavCmd::MAV_CMD_NAV_TAKEOFF, MavResult::DENIED);
                return; // ActionScope destructor returns to Hold.
            }

            // Immediate IN_PROGRESS ACK to prevent the QGC 500ms command timeout.
            this->send_command_ack(MavCmd::MAV_CMD_NAV_TAKEOFF, MavResult::IN_PROGRESS);

            const Outcome outcome = this->flight_primitives_.takeoff(
                scope, stoken,
                [this]() { this->send_command_ack(MavCmd::MAV_CMD_NAV_TAKEOFF, MavResult::IN_PROGRESS); }
            );

            if (const auto result = ack_for(outcome))
                this->send_command_ack(MavCmd::MAV_CMD_NAV_TAKEOFF, *result);

            if (outcome == Outcome::Done)
                scope.succeed();
        }
    );

    if (!started)
        this->send_command_ack(MavCmd::MAV_CMD_NAV_TAKEOFF, MavResult::TEMPORARILY_REJECTED);
}

void FlightActionExecutor::execute_land()
{
    // The heartbeat echoes the Land mode back as a mode-change request: ignore it while landing.
    if (this->state_manager_.get_active_maneuver() == Maneuver::Land)
        return;

    this->maneuver_runner_.run(
        Maneuver::Land, Policy::PreemptCurrent,
        [this](std::stop_token stoken, Scope& scope)
        {
            if (!this->dji_.is_armed())
            {
                this->ground_station_logger_.send_message(MavSeverity::WARNING, "Land refused: drone is not armed");
                return;
            }

            const Outcome outcome = this->flight_primitives_.land(scope, stoken);

            if (outcome == Outcome::Done)
                scope.succeed();
            else if (outcome != Outcome::Cancelled)
                this->ground_station_logger_.send_message(MavSeverity::ERROR, std::format("Landing {}", to_string(outcome)));
        }
    );
}

void FlightActionExecutor::execute_return_to_launch()
{
    // Same echo protection as the landing.
    if (this->state_manager_.get_active_maneuver() == Maneuver::ReturnToLaunch)
        return;

    this->maneuver_runner_.run(
        Maneuver::ReturnToLaunch, Policy::PreemptCurrent,
        [this](std::stop_token stoken, Scope& scope)
        {
            if (!this->dji_.is_armed())
            {
                this->ground_station_logger_.send_message(MavSeverity::WARNING, "RTL refused: drone is not armed");
                return;
            }

            const Outcome outcome = this->flight_primitives_.return_to_launch(scope, stoken);

            if (outcome == Outcome::Done)
                scope.succeed();
            else if (outcome != Outcome::Cancelled)
                this->ground_station_logger_.send_message(MavSeverity::ERROR, std::format("RTL {}", to_string(outcome)));
        }
    );
}

void FlightActionExecutor::execute_change_altitude(float target_altitude_amsl)
{
    const bool started = this->maneuver_runner_.run(
        Maneuver::Reposition, Policy::RejectIfBusy,
        [this, target_altitude_amsl](std::stop_token stoken, Scope& scope)
        {
            if (!this->dji_.is_armed() || this->dji_.is_manual_override_active())
            {
                this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::DENIED);
                return;
            }

            this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::IN_PROGRESS);

            const float target_altitude_rel = target_altitude_amsl - static_cast<float>(this->dji_.get_home_altitude());

            const Outcome outcome = this->flight_primitives_.change_altitude(
                scope, target_altitude_rel, stoken,
                [this]() { this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::IN_PROGRESS); }
            );

            if (const auto result = ack_for(outcome))
                this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, *result);

            if (outcome == Outcome::Done)
                scope.succeed();
        }
    );

    if (!started)
        this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::TEMPORARILY_REJECTED);
}

void FlightActionExecutor::execute_navigate_to_waypoint(const Location3D& waypoint)
{
    WaypointCommand command;
    command.target       = waypoint;
    command.speed_m_s    = 5.0f;
    command.heading_mode = HeadingMode::NoseForward;

    const bool started = this->maneuver_runner_.run(
        Maneuver::Reposition, Policy::RejectIfBusy,
        [this, command](std::stop_token stoken, Scope& scope)
        {
            if (!this->dji_.is_armed() || this->dji_.is_manual_override_active())
            {
                this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::DENIED);
                return;
            }

            this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::IN_PROGRESS);

            const Outcome outcome = this->flight_primitives_.go_to_waypoint(
                scope, command, stoken,
                [this]() { this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::IN_PROGRESS); }
            );

            if (const auto result = ack_for(outcome))
                this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, *result);

            if (outcome == Outcome::Done)
                scope.succeed();
        }
    );

    if (!started)
        this->send_command_ack(MavCmd::MAV_CMD_DO_REPOSITION, MavResult::TEMPORARILY_REJECTED);
}


/* Private Class Methods -----------------------------------------------------*/

void FlightActionExecutor::send_command_ack(uint16_t command, uint8_t result)
{
    this->mavlink_direct_server_.send_message(
        create_ack_mavlink_msg(command, result)
    );
}