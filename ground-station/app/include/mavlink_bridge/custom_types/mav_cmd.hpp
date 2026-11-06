/**
 ******************************************************************************
 * @file    mav_cmd.hpp
 * @brief   Definitions for MAVLink command IDs (MAV_CMD).
 *
 * This file contains constexpr definitions for standard MAVLink command
 * identifiers used throughout the MAVLink bridge to interpret and route
 * incoming Ground Control Station requests.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include <cstdint>


/* Definitions ---------------------------------------------------------------*/

/**
 * @namespace MavCmd
 * @brief Standard MAVLink command (MAV_CMD) identifiers.
 */
namespace MavCmd
{
    constexpr uint32_t MAV_CMD_NAV_WAYPOINT         = 16;  ///< Command to navigate to a waypoint.
    constexpr uint32_t MAV_CMD_NAV_LOITER_UNLIM     = 17;
    constexpr uint32_t MAV_CMD_NAV_LOITER_TURNS     = 18;
    constexpr uint32_t MAV_CMD_NAV_LOITER_TIME      = 19;
    constexpr uint32_t MAV_CMD_NAV_RETURN_TO_LAUNCH = 20;
    constexpr uint32_t MAV_CMD_NAV_LAND             = 21;  ///< Command to land at current location.
    constexpr uint32_t MAV_CMD_NAV_TAKEOFF          = 22;  ///< Command to takeoff from current location.
    constexpr uint32_t MAV_CMD_NAV_LAND_LOCAL       = 23;
    constexpr uint32_t MAV_CMD_NAV_TAKEOFF_LOCAL    = 24;
    constexpr uint32_t MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30;
    constexpr uint32_t MAV_CMD_NAV_LOITER_TO_ALT    = 31;
    constexpr uint32_t MAV_CMD_DO_FOLLOW            = 32;
    constexpr uint32_t MAV_CMD_DO_FOLLOW_REPOSITION = 33;
    constexpr uint32_t MAV_CMD_NAV_DELAY            = 93;
    constexpr uint32_t MAV_CMD_MISSION_START        = 300; ///< Command to start (or resume) the mission.
    constexpr uint32_t MAV_CMD_COMPONENT_ARM_DISARM = 400; ///< Command to arm or disarm the vehicle.
    constexpr uint32_t MAV_CMD_DO_REPOSITION        = 192; ///< Command to reposition the vehicle (change altitude/location).
    constexpr uint32_t MAV_CMD_DO_CHANGE_SPEED      = 178; ///< Command to change the target speed of the vehicle.
}