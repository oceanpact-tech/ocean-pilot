#pragma once

/* Includes ------------------------------------------------------------------*/

#include <cstdint>


/* Definitions ---------------------------------------------------------------*/

/**
 * @namespace PX4FlightModes
 * @brief Standard MAVLink flight mode identifiers.
 */
namespace PX4FlightModes 
{
    constexpr uint32_t PX4_CUSTOM_MODE_POSCTL           = 196608;    ///< Position control mode.
    constexpr uint32_t PX4_CUSTOM_MODE_HOLD             = 50593792;  ///< Hold mode.
    constexpr uint32_t PX4_CUSTOM_MODE_MISSION          = 67371008;  ///< Mission mode.
    constexpr uint32_t PX4_CUSTOM_MODE_RETURN_TO_LAUNCH = 84148224;  ///< Return to launch mode.
    constexpr uint32_t PX4_CUSTOM_MODE_LAND             = 100925440; ///< Land mode.
}
