/**
 ******************************************************************************
 * @file    ground_station_logger.hpp
 * @brief   Provides a unified interface for sending text logs to ground station.
 *
 * This file defines the GroundStationLogger class, which wraps the 
 * MavlinkDirectServer to easily dispatch STATUSTEXT messages. It handles 
 * MAVLink's 50-character string truncation requirements and JSON formatting 
 * automatically.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include <string>
#include <cstdint>

// MAVSDK Plugins
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>

// Composed modules
#include "mavlink_bridge/module_fwd.hpp"

// Mavlink types
#include "mavlink_bridge/custom_types/mav_severity.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Utility class to dispatch MAVLink STATUSTEXT messages to the GS.
 * 
 * Module references are stored for future extensions and must not be used 
 * during construction or destruction.
 */
class GroundStationLogger 
{
public:

    /**
     * @brief Constructor for GroundStationLogger.
     * @param mavlink_direct_server  Reference to the MAVSDK direct message plugin (mavsdk server).
     * @param state_manager          Reference to the flight state manager (composed module).
     * @param flight_action_executor Reference to the flight action executor (composed module).
     * @param flight_primitives      Reference to the flight primitives (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param mission_executor       Reference to the mission executor (composed module).
     * @param manual_control_handler Reference to the manual control handler (composed module).
     * @param telemetry_publisher    Reference to the telemetry publisher (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     */
    GroundStationLogger(
        // MAVSDK API
        mavsdk::MavlinkDirectServer& mavlink_direct_server,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives&     flight_primitives,
        MissionHandler&       mission_handler,
        MissionExecutor&      mission_executor,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher&   telemetry_publisher,
        ManeuverRunner&       maneuver_runner
    );

    /// @brief Destructor for GroundStationLogger.
    ~GroundStationLogger() = default;

    /**
     * @brief Sends a formatted text message to Ground Station.
     * @param severity The MAVSeverity level (determines UI color and audio).
     * @param message  The text payload (automatically truncated to 49 chars).
     */
    void send_message(uint8_t severity, const std::string& message) const;

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    mavsdk::MavlinkDirectServer& mavlink_direct_server_; ///< MAVSDK direct server for dispatching custom messages.


    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    FlightStateManager&   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    FlightActionExecutor& flight_action_executor_; ///< Entry points for takeoff, land, rtl, change altitude and go-to.
    FlightPrimitives&     flight_primitives_;      ///< Blocking and cancellable flight operations.
    MissionHandler&       mission_handler_;        ///< Mission plan storage and progress reporter.
    MissionExecutor&      mission_executor_;       ///< Mission interpreter with pause and resume.
    ManualControlHandler& manual_control_handler_; ///< Handler for DJI Virtual Stick and MANUAL_CONTROL inputs.
    TelemetryPublisher&   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.
};