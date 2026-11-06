/**
 ******************************************************************************
 * @file    mav_severity.hpp
 * @brief   Definitions for MAVLink STATUSTEXT severity levels.
 *
 * This file contains constexpr definitions for the MAV_SEVERITY enum 
 * used in STATUSTEXT messages to dictate the color and audio alerts 
 * presented to the pilot on the Ground Control Station.
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
 * @namespace MavSeverity
 * @brief Standard MAVLink severity levels for STATUSTEXT messages.
 */
namespace MavSeverity
{
    constexpr uint8_t EMERGENCY = 0; ///< System is unusable. (Red + Critical Alert)
    constexpr uint8_t ALERT     = 1; ///< Action must be taken immediately.
    constexpr uint8_t CRITICAL  = 2; ///< Critical conditions.
    constexpr uint8_t ERROR     = 3; ///< Error conditions. (Red text)
    constexpr uint8_t WARNING   = 4; ///< Warning conditions. (Yellow text)
    constexpr uint8_t NOTICE    = 5; ///< Normal but significant condition.
    constexpr uint8_t INFO      = 6; ///< Informational messages. (Normal text)
    constexpr uint8_t DEBUG     = 7; ///< Debug-level messages.
}