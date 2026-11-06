/**
 ******************************************************************************
 * @file    mission_item_utils.hpp
 * @brief   Stateless conversions from MAVLink mission items to flight commands.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <cstddef>
#include <cstdint>
#include <optional>

// MAVSDK
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.hpp>

#include "mavlink_bridge/flight_primitives.hpp"


/* Functions  ----------------------------------------------------------------*/

/**
 * @brief Checks whether a MAV_CMD can be executed by the MissionExecutor.
 * @param command The MAV_CMD identifier of the mission item.
 * @return true if the command is supported.
 */
bool is_supported_mission_command(uint16_t command);

/**
 * @brief Checks whether the first executable item at or after an index is a takeoff.
 * @param plan        Mission plan.
 * @param first_index Index where execution starts.
 * @return true if that item is MAV_CMD_NAV_TAKEOFF.
 */
bool starts_with_takeoff(const mavsdk::MissionRawServer::MissionPlan& plan, std::size_t first_index);

/**
 * @brief Converts a NAV_WAYPOINT item into a resolved navigation command.
 * 
 * Converts x/y from degrees * 1e7 to degrees, resolves the altitude to meters 
 * relative to home according to the item frame (0, 3, 5 and 6 are supported), 
 * and maps param4 (yaw) when it is not NaN.
 * 
 * @param item                 Mission item.
 * @param home_altitude_amsl_m Home altitude in meters AMSL.
 * @param speed_m_s            Current cruise speed set by DO_CHANGE_SPEED.
 * @return The command, or std::nullopt if the frame is unsupported.
 */
std::optional<WaypointCommand> to_waypoint_command(
    const mavsdk::MissionRawServer::MissionItem& item,
    float home_altitude_amsl_m,
    float speed_m_s
);