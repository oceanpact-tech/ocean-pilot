/**
 ******************************************************************************
 * @file    mav_land_state.hpp
 * @brief   Definitions for MAVLink landed states.
 *
 * This file contains constexpr definitions for the MAV_LANDED_STATE enum 
 * values used in the EXTENDED_SYS_STATE messages to report the current 
 * flight phase (grounded, airborne, taking off, or landing).
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
 * @namespace MavLandState
 * @brief Standard MAVLink landed state identifiers.
 */
namespace MavLandState 
{
    constexpr uint8_t MAV_LANDED_STATE_ON_GROUND = 1; ///< Vehicle is on the ground.
    constexpr uint8_t MAV_LANDED_STATE_IN_AIR    = 2; ///< Vehicle is flying in the air.
    constexpr uint8_t MAV_LANDED_STATE_TAKEOFF   = 3; ///< Vehicle is in the takeoff phase.
    constexpr uint8_t MAV_LANDED_STATE_LANDING   = 4; ///< Vehicle is in the landing phase.
}