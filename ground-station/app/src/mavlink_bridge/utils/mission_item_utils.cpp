/**
 ******************************************************************************
 * @file    mission_item_utils.cpp
 * @brief   Implementation of the MAVLink mission item conversions.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/utils/mission_item_utils.hpp"

#include <cmath>

#include "mavlink_bridge/custom_types/mav_cmd.hpp"
#include "mavlink_bridge/custom_types/mav_frame.hpp"


/* Public Functions ----------------------------------------------------------*/

bool is_supported_mission_command(uint16_t command)
{
    switch (command)
    {
        case MavCmd::MAV_CMD_NAV_WAYPOINT:
        case MavCmd::MAV_CMD_NAV_TAKEOFF:
        case MavCmd::MAV_CMD_NAV_LAND:
        case MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MavCmd::MAV_CMD_DO_CHANGE_SPEED:
        case MavCmd::MAV_CMD_NAV_DELAY:
        case MavCmd::MAV_CMD_NAV_LOITER_TIME:
            return true;

        default:
            return false;
    }
}

bool starts_with_takeoff(const mavsdk::MissionRawServer::MissionPlan& plan, std::size_t first_index)
{
    for (std::size_t i = first_index; i < plan.mission_items.size(); i++)
    {
        const auto command = plan.mission_items[i].command;

        // Items that do not move the drone are skipped.
        if (command == MavCmd::MAV_CMD_DO_CHANGE_SPEED || !is_supported_mission_command(command))
            continue;

        return command == MavCmd::MAV_CMD_NAV_TAKEOFF;
    }

    return false;
}

std::optional<WaypointCommand> to_waypoint_command(
    const mavsdk::MissionRawServer::MissionItem& item,
    float home_altitude_amsl_m,
    float speed_m_s
)
{
    float altitude_rel_m = 0.0f;

    switch (item.frame)
    {
        case MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            altitude_rel_m = item.z;
            break;

        case MavFrame::MAV_FRAME_GLOBAL:
        case MavFrame::MAV_FRAME_GLOBAL_INT:
            altitude_rel_m = item.z - home_altitude_amsl_m;
            break;

        default:
            return std::nullopt; // Terrain-relative and local frames are not supported.
    }

    WaypointCommand command;
    command.target.latitude  = item.x / 1.0e7;
    command.target.longitude = item.y / 1.0e7;
    command.target.altitude  = altitude_rel_m;
    command.speed_m_s        = speed_m_s;
    command.heading_mode     = HeadingMode::NoseForward;

    // param4 = yaw at the waypoint; NaN means "keep the current heading".
    if (!std::isnan(item.param4))
        command.yaw_deg = item.param4;

    return command;
}