#pragma once

#include <cstdint>

namespace MavLandState 
{
    constexpr uint8_t MAV_LANDED_STATE_ON_GROUND = 1;
    constexpr uint8_t MAV_LANDED_STATE_IN_AIR    = 2;
    constexpr uint8_t MAV_LANDED_STATE_TAKEOFF   = 3;
    constexpr uint8_t MAV_LANDED_STATE_LANDING   = 4;
}