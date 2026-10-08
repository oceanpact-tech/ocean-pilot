/**
 ******************************************************************************
 * @file    px4_params.hpp
 * @brief   Default PX4 parameter configurations for the MAVLink bridge.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include <utility>

/* Namespaces ----------------------------------------------------------------*/

/**
 * @brief Contains default parameters to simulate a PX4 autopilot.
 */
namespace oceanpilot::Params 
{
    /// @brief Integer parameters (System ID, RC mapping, Flight modes, etc).
    inline constexpr std::pair<const char*, int> int_params[] = {
        {"SYS_AUTOSTART", 4001}, 
        {"SYS_AUTOCONFIG", 0}, 
        {"MAV_SYS_ID", 1},
        {"COM_RC_IN_MODE", 0},
        {"RC_MAP_ROLL", 0}, 
        {"RC_MAP_PITCH", 0}, 
        {"RC_MAP_YAW", 0}, 
        {"RC_MAP_THROTTLE", 0},
        {"RC_MAP_FLAPS", 0}, 
        {"RC_MAP_AUX1", 0}, 
        {"RC_MAP_AUX2", 0},
        {"COM_FLTMODE1", -1}, 
        {"COM_FLTMODE2", -1}, 
        {"COM_FLTMODE3", -1},
        {"COM_FLTMODE4", -1}, 
        {"COM_FLTMODE5", -1}, 
        {"COM_FLTMODE6", -1},
        {"NAV_DLL_ACT", 0}, 
        {"NAV_RCL_ACT", 0}, 
        {"COM_LOW_BAT_ACT", 0},
        {"CAL_GYRO0_ID", 1}, 
        {"CAL_ACC0_ID", 1},
        {"CAL_MAG0_ID", 1}, 
        {"CAL_MAG1_ID", 1}, 
        {"CAL_MAG2_ID", 1},
    };

    /// @brief Float parameters (RTL altitudes, delays, and failsafe timeouts).
    inline constexpr std::pair<const char*, float> float_params[] = {
        {"RTL_LAND_DELAY", 0.0f},  
        {"RTL_DESCEND_ALT", 30.0f},
        {"RTL_RETURN_ALT", 60.0f}, 
        {"COM_RC_LOSS_T", 0.5f},
    };
}