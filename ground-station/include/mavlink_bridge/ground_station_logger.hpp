/**
 ******************************************************************************
 * @file    ground_station_logger.hpp
 * @brief   Provides a unified interface for sending text logs to ground station.
 *
 * This file defines the QgcLogger class, which wraps the MavlinkDirectServer
 * to easily dispatch STATUSTEXT messages. It handles MAVLink's 50-character
 * string truncation requirements and JSON formatting automatically.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include <string>
#include <cstdint>

// MAVSDK
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>

#include "mavlink_bridge/custom_types/mav_severity.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Utility class to dispatch MAVLink STATUSTEXT messages to the GS.
 */
class GroundStationLogger 
{
public:

    /**
     * @brief Constructor for GroundStationLogger.
     * @param direct_server Reference to the MAVSDK direct message plugin.
     */
    explicit GroundStationLogger(mavsdk::MavlinkDirectServer& direct_server);

    /// @brief Destructor for GroundStationLogger.
    ~GroundStationLogger() = default;

    /**
     * @brief Sends a formatted text message to Ground Station.
     * @param severity The MAVSeverity level (determines UI color and audio).
     * @param message  The text payload (automatically truncated to 49 chars).
     */
    void send_message(uint8_t severity, const std::string& message) const;

private:

    mavsdk::MavlinkDirectServer& direct_server_; ///< MAVSDK direct server for dispatching custom messages.
};