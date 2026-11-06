/**
 ******************************************************************************
 * @file    mission_handler.cpp
 * @brief   Implementation of the mission plan storage and progress reporter.
 *
 * This file implements the MissionHandler class. It keeps the latest valid 
 * mission plan received from the Ground Station and reports execution progress. 
 * Execution itself lives in MissionExecutor.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/mission_handler.hpp"

#include <format>
#include <iostream>

// Composed modules
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/flight_primitives.hpp"
#include "mavlink_bridge/flight_action_executor.hpp"
#include "mavlink_bridge/mission_executor.hpp"
#include "mavlink_bridge/ground_station_logger.hpp"
#include "mavlink_bridge/manual_control_handler.hpp"
#include "mavlink_bridge/telemetry_publisher.hpp"
#include "mavlink_bridge/maneuver_runner.hpp"

// Mavlink types
#include "mavlink_bridge/custom_types/mav_severity.hpp"

// Helpers
#include "mavlink_bridge/utils/mission_item_utils.hpp"


/* Local definitions ---------------------------------------------------------*/

namespace
{
    constexpr uint32_t MAV_MISSION_TYPE_MISSION = 0;   ///< Waypoint mission.
    constexpr uint32_t MAV_MISSION_TYPE_ALL     = 255; ///< Every mission type (used by clear-all).
}


/* Public Class Methods ------------------------------------------------------*/

MissionHandler::MissionHandler(
    // MAVSDK API
    mavsdk::MissionRawServer& mission_server,
    // Composed modules
    FlightStateManager&   state_manager,
    FlightActionExecutor& flight_action_executor,
    FlightPrimitives&     flight_primitives,
    MissionExecutor&      mission_executor,
    GroundStationLogger&  ground_station_logger,
    ManualControlHandler& manual_control_handler,
    TelemetryPublisher&   telemetry_publisher,
    ManeuverRunner&       maneuver_runner
)
    : mission_server_(mission_server),
      state_manager_(state_manager),
      flight_action_executor_(flight_action_executor),
      flight_primitives_(flight_primitives),
      mission_executor_(mission_executor),
      ground_station_logger_(ground_station_logger),
      manual_control_handler_(manual_control_handler),
      telemetry_publisher_(telemetry_publisher),
      maneuver_runner_(maneuver_runner)
{
}

MissionHandler::~MissionHandler() 
{
    this->mission_server_.unsubscribe_incoming_mission(this->incoming_mission_handle_);
    this->mission_server_.unsubscribe_clear_all(this->clear_all_handle_);
}

void MissionHandler::start() 
{
    this->incoming_mission_handle_ = this->mission_server_.subscribe_incoming_mission(
        [this](Result result, MissionPlan plan) 
        {
            this->on_incoming_mission(result, std::move(plan));
        }
    );

    this->clear_all_handle_ = this->mission_server_.subscribe_clear_all(
        [this](uint32_t mission_type) 
        {
            this->on_clear_all(mission_type);
        }
    );
}

std::optional<MissionHandler::PlanSnapshot> MissionHandler::get_plan() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->plan_)
        return std::nullopt;

    return PlanSnapshot{*this->plan_, this->version_};
}

bool MissionHandler::has_mission() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->plan_.has_value();
}


/* Private Class Methods -----------------------------------------------------*/

void MissionHandler::on_incoming_mission(Result result, MissionPlan plan)
{
    if (result != Result::Success) 
    {
        std::cerr << "[MissionHandler] Failure to receive mission from QGC: " << result << '\n';
        this->ground_station_logger_.send_message(MavSeverity::ERROR, "Mission upload failed");
        return;
    }

    // Fence and rally transfers use the same callback: keep only waypoint missions.
    MissionPlan mission;
    int unsupported_items = 0;

    for (const auto& item : plan.mission_items)
    {
        if (item.mission_type != MAV_MISSION_TYPE_MISSION)
            continue;

        if (!is_supported_mission_command(static_cast<uint16_t>(item.command)))
            unsupported_items++;

        mission.mission_items.push_back(item);
    }

    // Empty and non-mission transfers do not replace the stored plan (clearing uses clear-all).
    if (mission.mission_items.empty())
        return;

    const std::size_t item_count = mission.mission_items.size();

    {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->plan_ = std::move(mission);
        this->version_++;
    }

    this->mission_executor_.reset_progress();

    this->ground_station_logger_.send_message(MavSeverity::INFO, std::format("Mission stored: {} items", item_count));

    if (unsupported_items > 0)
        this->ground_station_logger_.send_message(MavSeverity::WARNING, std::format("{} unsupported item(s) will be skipped", unsupported_items));
}

void MissionHandler::on_clear_all(uint32_t mission_type)
{
    if (mission_type != MAV_MISSION_TYPE_MISSION && mission_type != MAV_MISSION_TYPE_ALL)
        return;

    {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->plan_.reset();
        this->version_++;
    }

    this->mission_executor_.reset_progress();
}