/**
 ******************************************************************************
 * @file    manual_control_handler.hpp
 * @brief   Manages DJI Virtual Stick lifecycle and QGC MANUAL_CONTROL inputs.
 *
 * This file defines the ManualControlHandler class, which bridges QGroundControl
 * joystick inputs (MANUAL_CONTROL messages) to the DJI Virtual Stick API. It 
 * continuously synchronizes the Virtual Stick enable/disable state with the 
 * FlightStateManager and enforces a 20 Hz rate limit on stick command dispatch.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <atomic>
#include <chrono>

// DJI
#include "dji_interface/dji_interface.hpp"

// MAVSDK API
#include <mavsdk/mavsdk.hpp>

// Composed modules
#include "mavlink_bridge/module_fwd.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Controller for Virtual Stick state management and manual joystick inputs.
 * 
 * The ManualControlHandler monitors pilot manual overrides, automatically 
 * enables or disables the DJI Virtual Stick based on the active flight mode 
 * and maneuver phase, and scales incoming MAVLink MANUAL_CONTROL axes before 
 * sending them to the drone. MANUAL_CONTROL frames are ignored while a 
 * maneuver is active.
 * 
 * Module references are stored for future extensions and must not be used 
 * during construction or destruction.
 */
class ManualControlHandler 
{
public:

    /**
     * @brief Constructor for ManualControlHandler.
     * @param dji                    Reference to the DJI hardware interface.
     * @param state_manager          Reference to the flight state manager (composed module).
     * @param flight_action_executor Reference to the flight action executor (composed module).
     * @param flight_primitives      Reference to the flight primitives (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param mission_executor       Reference to the mission executor (composed module).
     * @param ground_station_logger  Reference to the ground station logger (composed module).
     * @param telemetry_publisher    Reference to the telemetry publisher (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     */
    ManualControlHandler(
        // DJI API
        DJIInterface& dji,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives&     flight_primitives,
        MissionHandler&       mission_handler,
        MissionExecutor&      mission_executor,
        GroundStationLogger&  ground_station_logger,
        TelemetryPublisher&   telemetry_publisher,
        ManeuverRunner&       maneuver_runner
    );

    /// @brief Destructor for ManualControlHandler.
    ~ManualControlHandler() = default;

    /**
     * @brief Synchronizes the DJI Virtual Stick state with the FlightStateManager.
     * 
     * Detects pilot manual overrides, disables Virtual Stick when it is not 
     * required (outside Posctl and outside navigation phases), and periodically 
     * retries enabling Virtual Stick every 1.5 seconds when required.
     */
    void maintain_virtual_stick();

    /**
     * @brief Parses and forwards incoming MANUAL_CONTROL messages to the DJI interface.
     * @param msg The raw MAVLink MANUAL_CONTROL message received from the Ground Station.
     * @return true to allow further MAVSDK processing, false if malformed.
     */
    bool handle_manual_control_message(const mavsdk::Mavsdk::MavlinkMessage& msg);

    /**
     * @brief Updates the scale divider applied to normalized stick inputs.
     * @param divider Scale factor (e.g., 1.0 for full range, 2.0 for half range).
     */
    void set_stick_scale_divider(float divider) noexcept;

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    DJIInterface& dji_; ///< DJI interface for telemetry polling and command dispatch.


    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    FlightStateManager&   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    FlightActionExecutor& flight_action_executor_; ///< Entry points for takeoff, land, rtl, change altitude and go-to.
    FlightPrimitives&     flight_primitives_;      ///< Blocking and cancellable flight operations.
    MissionHandler&       mission_handler_;        ///< Mission plan storage and progress reporter.
    MissionExecutor&      mission_executor_;       ///< Mission interpreter with pause and resume.
    GroundStationLogger&  ground_station_logger_;  ///< Logger utility for dispatching STATUSTEXT messages and alerts to the GCS.
    TelemetryPublisher&   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.


    // --------------------------
    // INTERNAL STATE & TIMERS
    // --------------------------

    std::atomic<bool>  is_virtual_stick_enabling_{false}; ///< Flag indicating an in-flight HTTP enable request.
    std::atomic<float> stick_scale_divider_{2.0f};        ///< Sensitivity divider for stick axes.

    std::chrono::steady_clock::time_point last_enable_attempt_; ///< Timestamp of last enableVirtualStick HTTP call.
    std::chrono::steady_clock::time_point last_stick_send_;     ///< Timestamp of last stick payload dispatch (20 Hz).
};