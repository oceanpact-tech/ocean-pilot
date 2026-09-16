#pragma once

#include <string_view>

namespace endpoints {
    constexpr std::string_view TAKEOFF = "/send/takeoff";
    constexpr std::string_view LAND = "/send/land";
    constexpr std::string_view GOTO_YAW = "/send/gotoYaw";
    constexpr std::string_view GOTO_WP_NOSE_FORWARD = "/send/gotoWaypointNoseForward";
    constexpr std::string_view ALL_STATES = "/aircraft/allStates";
}