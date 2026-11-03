/**
 ******************************************************************************
 * @file    flight_state_manager.hpp
 * @brief   Thread-safe state machine for flight modes and active maneuvers.
 *
 * This file defines the FlightStateManager class, which acts as the single 
 * source of truth for the drone's current flight mode, active autonomous 
 * maneuvers (Takeoff, Land), and Virtual Stick intent. It uses an RAII scope 
 * guard (ActionScope) to guarantee state consistency across concurrent threads.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <mutex>
#include <optional>
#include <utility>
#include <cstdint>

// MAVSDK
#include <mavsdk/plugins/action_server/action_server.hpp>

#include "mavlink_bridge/mission_handler.hpp"
#include "mavlink_bridge/custom_types/mav_land_state.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Centralized, thread-safe coordinator for flight modes and maneuvers.
 * 
 * The FlightStateManager encapsulates all state variables that govern mode 
 * transitions and Virtual Stick activation. It eliminates race conditions 
 * between telemetry publishing, GCS mode requests, and asynchronous action 
 * threads by protecting state transitions under a single mutex.
 */
class FlightStateManager 
{
public:

    using FlightMode = mavsdk::ActionServer::FlightMode;

    /// @brief Enumeration of mutually exclusive autonomous maneuvers.
    enum class ActiveManeuver {
        None,          ///< No autonomous maneuver in progress.
        Takeoff,       ///< Drone is currently executing automatic takeoff.
        Land,          ///< Drone is currently executing automatic landing.
        ReturnToLaunch ///< Drone is currenty executing automatic return to home.
    };

    /**
     * @brief RAII scope guard for asynchronous maneuvers (Takeoff and Land).
     * 
     * Lives on the worker thread executing the maneuver. When destroyed, it 
     * automatically restores the previous flight mode if succeed() was not 
     * called, or transitions cleanly to Hold mode upon successful completion.
     */
    class ActionScope 
    {
    public:

        ActionScope(const ActionScope&) = delete;
        ActionScope& operator=(const ActionScope&) = delete;

        /**
         * @brief Move constructor for ActionScope.
         * @param other The source ActionScope to move from.
         */
        ActionScope(ActionScope&& other) noexcept
            : manager_(std::exchange(other.manager_, nullptr)),
              maneuver_(other.maneuver_),
              mode_before_(other.mode_before_),
              succeeded_(other.succeeded_) {}

        /// @brief Destructor for ActionScope. Finalizes maneuver state in FlightStateManager.
        ~ActionScope() 
        {
            if (this->manager_) {
                this->manager_->finish_maneuver(this->maneuver_, this->mode_before_, this->succeeded_);
            }
        }

        /// @brief Marks the maneuver as completed successfully, forcing transition to Hold.
        void succeed() noexcept { this->succeeded_ = true; }

    private:

        friend class FlightStateManager;

        /**
         * @brief Private constructor accessible only by FlightStateManager.
         * @param manager     Pointer to the owning FlightStateManager.
         * @param maneuver    The maneuver being started.
         * @param mode_before The flight mode active prior to starting the maneuver.
         */
        ActionScope(
            FlightStateManager* manager, 
            ActiveManeuver maneuver, 
            FlightMode mode_before
        )
            : manager_(manager), 
              maneuver_(maneuver), 
              mode_before_(mode_before) {}

        FlightStateManager* manager_{nullptr};               ///< Pointer to owning state manager.
        ActiveManeuver      maneuver_{ActiveManeuver::None}; ///< Active maneuver tied to this scope.
        FlightMode          mode_before_{FlightMode::Hold};  ///< Flight mode to restore on abort.
        bool                succeeded_{false};               ///< Flag indicating normal completion.
    };

    /// @brief Default constructor for FlightStateManager.
    FlightStateManager(
        MissionHandler& mission_handler,
        FlightActionExecutor& flight_action_executor
    )
        : mission_handler_(mission_handler),
          flight_action_executor_(flight_action_executor)
    {}

    /// @brief Destructor for FlightStateManager.
    ~FlightStateManager() = default;

    /**
     * @brief Atomically attempts to start an autonomous maneuver.
     * @param maneuver The maneuver to start (Takeoff or Land).
     * @return std::optional<ActionScope> if no maneuver was active, or std::nullopt otherwise.
     */
    std::optional<ActionScope> try_begin_maneuver(ActiveManeuver maneuver);

    /**
     * @brief Processes a flight mode transition requested by the Ground Station.
     * @param requested_mode The target flight mode requested by the GCS.
     * @return true if the mode changed, false if already in the requested mode.
     */
    bool request_flight_mode(FlightMode requested_mode);

    /// @brief Reverts mode to Hold and disables Virtual Stick intent upon pilot manual override.
    void on_manual_override_active();

    /**
     * @brief Computes the target FlightMode to be published in the MAVLink Heartbeat.
     * @param is_armed                 Current armed state from DJI hardware.
     * @param is_virtual_stick_enabled Current Virtual Stick state from DJI hardware.
     * @return The effective FlightMode to report to the Ground Station.
     */
    FlightMode get_target_flight_mode(bool is_armed, bool is_virtual_stick_enabled) const;

    /**
     * @brief Computes the MAV_LANDED_STATE for the EXTENDED_SYS_STATE message.
     * @param is_armed Current armed state from DJI hardware.
     * @return The MAVLandState byte (ON_GROUND, IN_AIR, TAKEOFF, or LANDING).
     */
    uint8_t get_landed_state(bool is_armed) const;

    /**
     * @brief Checks whether the DJI Virtual Stick should currently be enabled.
     * @param is_armed Current armed state from DJI hardware.
     * @return true if in Posctl with no active maneuver and drone is armed.
     */
    bool should_virtual_stick_be_enabled(bool is_armed) const;

    /// @brief Returns the currently active maneuver (None, Takeoff, or Land).
    ActiveManeuver get_active_maneuver() const;

private:

    // Composed modules. 
    MissionHandler& mission_handler_; ///< Mission Handler; 
    FlightActionExecutor& flight_action_executor_; ///< Flight action executor

    /**
     * @brief Internal callback invoked by ActionScope destructor to exit a maneuver.
     * @param maneuver    The maneuver being finished.
     * @param mode_before The mode saved when the maneuver began.
     * @param succeeded   Whether the maneuver completed normally or aborted.
     */
    void finish_maneuver(ActiveManeuver maneuver, FlightMode mode_before, bool succeeded);

    mutable std::mutex mutex_;                                 ///< Mutex protecting all state variables.
    FlightMode         desired_mode_{FlightMode::Hold};        ///< Desired flight mode outside maneuvers.
    ActiveManeuver     active_maneuver_{ActiveManeuver::None}; ///< Currently running maneuver.
    bool               want_virtual_stick_{false};             ///< Flag indicating if current mode requires V-Stick.
};