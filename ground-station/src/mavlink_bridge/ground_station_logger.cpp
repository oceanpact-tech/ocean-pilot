/**
 ******************************************************************************
 * @file    ground_station_logger.cpp
 * @brief   Implementation of the ground station logger utility.
 *
 * This file implements the GroundStationLogger class, encapsulating the JSON 
 * payload construction for the STATUSTEXT MAVLink message.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/ground_station_logger.hpp"

// Composed modules
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/flight_primitives.hpp"
#include "mavlink_bridge/mission_handler.hpp"
#include "mavlink_bridge/mission_executor.hpp"
#include "mavlink_bridge/ground_station_logger.hpp"
#include "mavlink_bridge/manual_control_handler.hpp"
#include "mavlink_bridge/telemetry_publisher.hpp"
#include "mavlink_bridge/maneuver_runner.hpp"


/* Public Class Methods ------------------------------------------------------*/

GroundStationLogger::GroundStationLogger(
    // MAVSDK API
    mavsdk::MavlinkDirectServer& mavlink_direct_server,
    // Composed modules
    FlightStateManager& state_manager,
    FlightActionExecutor& flight_action_executor,
    FlightPrimitives& flight_primitives,
    MissionHandler& mission_handler,
    MissionExecutor& mission_executor,
    ManualControlHandler& manual_control_handler,
    TelemetryPublisher& telemetry_publisher,
    ManeuverRunner& maneuver_runner
)
    : // MAVSDK API
      mavlink_direct_server_(mavlink_direct_server),
      // Composed modules
      state_manager_(state_manager),
      flight_primitives_(flight_primitives),
      flight_action_executor_(flight_action_executor),
      mission_handler_(mission_handler),
      mission_executor_(mission_executor),
      manual_control_handler_(manual_control_handler),
      telemetry_publisher_(telemetry_publisher),
      maneuver_runner_(maneuver_runner)
{
}

void GroundStationLogger::send_message(uint8_t severity, const std::string& message) const
{
    // text limited to 50 characters
    std::string safe_msg = message.length() > 49 ? message.substr(0, 49) : message;

    mavsdk::MavlinkDirectServer::MavlinkMessage msg;
    msg.message_name = "STATUSTEXT";
    msg.fields_json = R"({
        "severity": )" + std::to_string(severity) + R"(,
        "text": ")" + safe_msg + R"(",
        "id": 0,
        "chunk_seq": 0
    })";

    this->mavlink_direct_server_.send_message(msg);
}