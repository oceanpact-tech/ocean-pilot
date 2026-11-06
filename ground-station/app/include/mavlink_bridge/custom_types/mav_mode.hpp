#pragma once

/* Includes ------------------------------------------------------------------*/

#include <cstdint>


/* Definitions ---------------------------------------------------------------*/

/**
 * @namespace MavMode
 * @brief Standard MAVLink mode (MAV_MODE) identifiers.
 */
namespace MavMode
{
    constexpr uint32_t MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;   ///< Custom mode enabled flag.
    constexpr uint32_t MAV_MODE_FLAG_SAFETY_ARMED        = 128; ///< Safety armed flag (bit 0 set, bit 4 set).
}