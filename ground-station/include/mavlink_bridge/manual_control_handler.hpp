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

// MAVSDK
#include <mavsdk/mavsdk.hpp>

#include "dji_interface/dji_interface.hpp"
#include "mavlink_bridge/flight_state_manager.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Controller for Virtual Stick state management and manual joystick inputs.
 * 
 * The ManualControlHandler monitors pilot manual overrides, automatically 
 * enables or disables the DJI Virtual Stick based on the active flight mode, 
 * and scales incoming MAVLink MANUAL_CONTROL axes before sending them to the drone.
 */
class ManualControlHandler 
{
public:

    /**
     * @brief Constructor for ManualControlHandler.
     * @param dji           Reference to the DJI hardware interface.
     * @param state_manager Reference to the flight state manager.
     */
    ManualControlHandler(
        DJIInterface& dji,
        FlightStateManager& state_manager
    );

    /// @brief Destructor for ManualControlHandler.
    ~ManualControlHandler() = default;

    /**
     * @brief Synchronizes the DJI Virtual Stick state with the FlightStateManager.
     * 
     * Detects pilot manual overrides, disables Virtual Stick when leaving Posctl, 
     * and periodically retries enabling Virtual Stick every 1.5 seconds when required.
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

    DJIInterface&       dji_;           ///< DJI interface for Virtual Stick control.
    FlightStateManager& state_manager_; ///< State manager for querying Virtual Stick intent.


    // --------------------------
    // INTERNAL STATE & TIMERS
    // --------------------------

    std::atomic<bool>  is_virtual_stick_enabling_{false}; ///< Flag indicating an in-flight HTTP enable request.
    std::atomic<float> stick_scale_divider_{2.0f};        ///< Sensitivity divider for stick axes.

    std::chrono::steady_clock::time_point last_enable_attempt_; ///< Timestamp of last enableVirtualStick HTTP call.
    std::chrono::steady_clock::time_point last_stick_send_;     ///< Timestamp of last stick payload dispatch (20 Hz).
};