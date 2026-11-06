/**
 ******************************************************************************
 * @file    mav_frame.hpp
 * @brief   MAV_FRAME constants used by mission items.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <cstdint>


/* Definitions  --------------------------------------------------------------*/

namespace MavFrame 
{
    constexpr uint8_t MAV_FRAME_GLOBAL                  = 0;  ///< Altitude is AMSL (latitude and longitude in floating point degrees).
    constexpr uint8_t MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3;  ///< Altitude is relative to home (latitude and longitude in floating point degrees).
    constexpr uint8_t MAV_FRAME_GLOBAL_INT              = 5;  ///< Altitude is AMSL (latitude and longitude in degrees * 1e7).
    constexpr uint8_t MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6;  ///< Altitude is relative to home (latitude and longitude in degrees * 1e7).
    constexpr uint8_t MAV_FRAME_GLOBAL_TERRAIN_ALT      = 10; ///< Altitude is relative to terrain (unsupported).
    constexpr uint8_t MAV_FRAME_GLOBAL_TERRAIN_ALT_INT  = 11; ///< Altitude is relative to terrain (unsupported).
}