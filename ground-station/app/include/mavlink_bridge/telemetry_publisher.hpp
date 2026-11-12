/**
 ******************************************************************************
 * @file    telemetry_publisher.hpp
 * @brief   Polls DJI telemetry and publishes MAVLink state, heartbeat and sensors.
 *
 * This file defines the TelemetryPublisher class, responsible for periodically
 * reading raw sensor data from the DJIInterface, converting coordinate frames,
 * and dispatching MAVLink telemetry, Heartbeat states, SYS_STATUS health bits,
 * and EXTENDED_SYS_STATE landed states to the Ground Control Station.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <atomic>

// MAVSDK
#include <mavsdk/plugins/telemetry_server/telemetry_server.hpp>
#include <mavsdk/plugins/action_server/action_server.hpp>
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>

#include "dji_interface/dji_interface.hpp"
#include "mavlink_bridge/module_fwd.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Manages the formatting and periodic publication of MAVLink telemetry and state.
 * 
 * The TelemetryPublisher acts as the sole writer to the MAVSDK TelemetryServer 
 * and ActionServer state setters (armed state and flight mode). It queries 
 * FlightStateManager and DJIInterface on every cycle to keep QGroundControl 
 * synchronized with the physical aircraft.
 * 
 * Module references are stored for future extensions and must not be used 
 * during construction or destruction.
 */
class TelemetryPublisher 
{
public:

    /**
     * @brief Constructor for TelemetryPublisher.
     * @param dji                    Reference to the DJI hardware interface.
     * @param telemetry_server       Reference to the MAVSDK telemetry server plugin.
     * @param action_server          Reference to the MAVSDK action server plugin.
     * @param mavlink_direct_server  Reference to the MAVSDK direct message plugin (mavsdk server).
     * @param gimbal_direct_server   Reference to the MAVSDK direct message plugin (mavsdk server) for gimbal.
     * @param state_manager          Reference to the flight state manager (composed module).
     * @param flight_action_executor Reference to the flight action executor (composed module).
     * @param flight_primitives      Reference to the flight primitives (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param mission_executor       Reference to the mission executor (composed module).
     * @param ground_station_logger  Reference to the ground station logger (composed module).
     * @param manual_control_handler Reference to the manual control handler (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     */
    TelemetryPublisher(
        // DJI API
        DJIInterface& dji,
        // MAVSDK API
        mavsdk::TelemetryServer& telemetry_server,
        mavsdk::ActionServer& action_server,
        mavsdk::MavlinkDirectServer& mavlink_direct_server,
        mavsdk::MavlinkDirectServer& gimbal_direct_server,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives&     flight_primitives,
        MissionHandler&       mission_handler,
        MissionExecutor&      mission_executor,
        GroundStationLogger&  ground_station_logger,
        ManualControlHandler& manual_control_handler,
        ManeuverRunner&       maneuver_runner
    );

    /// @brief Destructor for TelemetryPublisher.
    ~TelemetryPublisher() = default;

    /// @brief Executes a complete telemetry and heartbeat publishing cycle.
    void publish_cycle();

    /// @brief Forces the next heartbeat cycle to re-publish the current FlightMode to ActionServer.
    void invalidate_published_mode() noexcept;

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    DJIInterface&                dji_;                   ///< DJI interface for telemetry polling and command dispatch.
    mavsdk::TelemetryServer&     telemetry_server_;      ///< MAVSDK telemetry server.
    mavsdk::ActionServer&        action_server_;         ///< MAVSDK action server.
    mavsdk::MavlinkDirectServer& mavlink_direct_server_; ///< MAVSDK direct server for dispatching custom messages.
    mavsdk::MavlinkDirectServer& gimbal_direct_server_;  ///< MAVSDK direct server for dispatching custom messages for gimbal.


    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    FlightStateManager&   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    FlightActionExecutor& flight_action_executor_; ///< Entry points for takeoff, land, rtl, change altitude and go-to.
    FlightPrimitives&     flight_primitives_;      ///< Blocking and cancellable flight operations.
    MissionHandler&       mission_handler_;        ///< Mission plan storage and progress reporter.
    MissionExecutor&      mission_executor_;       ///< Mission interpreter with pause and resume.
    GroundStationLogger&  ground_station_logger_;  ///< Logger utility for dispatching STATUSTEXT messages and alerts to the GCS.
    ManualControlHandler& manual_control_handler_; ///< Handler for DJI Virtual Stick and MANUAL_CONTROL inputs.
    ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.


    // --------------------------
    // HEARTBEAT STATE TRACKING
    // --------------------------

    mavsdk::ActionServer::FlightMode last_published_mode_{
        mavsdk::ActionServer::FlightMode::Unknown
    }; ///< Last flight mode sent to ActionServer to prevent redundant updates.

    std::atomic<bool> force_mode_republish_{true}; ///< Flag to force mode update when GCS requests unsupported mode.


    // --------------------------
    // TELEMETRY BUFFERS
    // --------------------------

    // Raw buffers for telemetry
    Location2D raw_home_location_buffer_{};   ///< Raw home location 2D data from DJIInterface.
    double     raw_home_altitude_buffer_{0.0};///< Raw home location altitude data from DJIInterface.
    Location3D raw_location_buffer_{};        ///< Raw location data from DJIInterface.
    Speed3D    raw_speed_buffer_{};           ///< Raw speed data from DJIInterface.
    double     raw_heading_buffer_{0.0};      ///< Raw heading data from DJIInterface.
    Attitude3D raw_attitude_buffer_{};        ///< Raw attitude data from DJIInterface.

    // Buffers for MAVSDK telemetry
    mavsdk::TelemetryServer::Position            home_location_buffer_{};    ///< Buffer for home location telemetry.
    mavsdk::TelemetryServer::Position            position_buffer_{};         ///< Buffer for position telemetry.
    mavsdk::TelemetryServer::VelocityNed         velocity_buffer_{};         ///< Buffer for velocity telemetry.
    mavsdk::TelemetryServer::Heading             heading_buffer_{};          ///< Buffer for heading telemetry.
    mavsdk::TelemetryServer::EulerAngle          attitude_buffer_{};         ///< Buffer for attitude telemetry.
    mavsdk::TelemetryServer::AngularVelocityBody angular_velocity_buffer_{}; ///< Buffer for angular velocity telemetry.


    // --------------------------
    // PRIVATE METHODS
    // --------------------------

    /// @brief Get raw telemetry from DJI.
    void get_raw_telemetry();

    /// @brief Populate telemetry buffers with data received from DJI.
    void prepare_mavlink_telemetry();

    /// @brief Publish position, home, and attitude telemetry to MAVLink.
    void publish_telemetry();

    /// @brief Updates armed state and flight mode in the MAVLink Heartbeat.
    void publish_heartbeat();

    /// @brief Publishes SYS_STATUS message including pre-arm check health bit.
    void publish_sys_status();

    /// @brief Publishes EXTENDED_SYS_STATE message with current landed state.
    void publish_extended_sys_state();


    void publish_gimbal();
};