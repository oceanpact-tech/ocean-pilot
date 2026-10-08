#pragma once

#include <string>

/**
 * @brief Represents a 3D vector for velocities or coordinates.
 */
struct Speed3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief Represents a 3D vector for attitude.
 */
struct Attitude3D {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

/**
 * @brief Represents a 3D vector for location.
 */
struct Location3D {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
};

/**
 * @brief Represents a 2D vector for location.
 */
struct Location2D {
    double latitude = 0.0;
    double longitude = 0.0;
};


/**
 * @brief Network params to connect.
 */
struct WildBridgeSocketConfig {
    std::string ip;
    std::string command_port;
    std::string telemetry_port;
};