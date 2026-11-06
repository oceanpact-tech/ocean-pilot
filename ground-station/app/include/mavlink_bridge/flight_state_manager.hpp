/**
 ******************************************************************************
 * @file    flight_state_manager.hpp
 * @brief   Thread-safe state machine for flight modes and active maneuvers.
 *
 * This file defines the FlightStateManager class, which acts as the single 
 * source of truth for the drone's current flight mode, active autonomous 
 * maneuvers (Takeoff, Land, ReturnToLaunch, Reposition, Mission) and Virtual 
 * Stick intent. It uses an RAII scope guard (ActionScope) to guarantee state 
 * consistency across concurrent threads.
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

// MAVSDK API
#include <mavsdk/plugins/action_server/action_server.hpp>

// Mavlink types
#include "mavlink_bridge/custom_types/mav_land_state.hpp"

// Composed modules
#include "mavlink_bridge/module_fwd.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Centralized, thread-safe coordinator for flight modes and maneuvers.
 * 
 * The FlightStateManager encapsulates all state variables that govern mode 
 * transitions and Virtual Stick activation. It eliminates race conditions 
 * between telemetry publishing, GCS mode requests, and asynchronous action 
 * threads by protecting state transitions under a single mutex.
 * 
 * Exactly one maneuver can be active at a time. The reported flight mode is a 
 * property of the active maneuver, while the landed state and the Virtual Stick 
 * requirement also depend on its phase.
 * 
 * Lock-order rule: this class must never call another module while holding its 
 * mutex. Module references are stored for future extensions and must not be 
 * used during construction or destruction.
 */
class FlightStateManager 
{
public:

    using FlightMode = mavsdk::ActionServer::FlightMode;

    /// @brief Enumeration of mutually exclusive autonomous maneuvers.
    enum class ActiveManeuver {
        None,           ///< No autonomous maneuver in progress.
        Takeoff,        ///< Drone is currently executing automatic takeoff.
        Land,           ///< Drone is currently executing automatic landing.
        ReturnToLaunch, ///< Drone is currenty executing automatic return to home.
        Reposition,     ///< Altitude change or go-to. Keeps the flight mode reported before it started.
        Mission         ///< Mission execution. Reported mode stays Mission for its whole duration.
    };

    /// @brief Sub-phase of a maneuver. Drives the landed state and the Virtual Stick requirement.
    enum class Phase {
        None,           ///< No specific phase.
        Takeoff,        ///< Taking off (Virtual Stick must be disabled).
        Landing,        ///< Landing (Virtual Stick must be disabled).
        ReturnToLaunch, ///< Returning home (Virtual Stick must be disabled).
        Navigating      ///< Flying to a waypoint or changing altitude (Virtual Stick must be enabled).
    };

    /**
     * @brief RAII scope guard for asynchronous maneuvers.
     * 
     * Lives on the worker thread executing the maneuver. When destroyed, it 
     * releases the maneuver slot and transitions to Hold, whether the maneuver 
     * succeeded or not. Reposition is the exception: it restores the mode that 
     * was active before it started (unless the pilot took over via RC).
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
              succeeded_(other.succeeded_) {}

        /// @brief Destructor for ActionScope. Finalizes maneuver state in FlightStateManager.
        ~ActionScope() 
        {
            if (this->manager_) {
                this->manager_->finish_maneuver(this->maneuver_, this->succeeded_);
            }
        }

        /// @brief Marks the maneuver as completed successfully.
        void succeed() noexcept { this->succeeded_ = true; }

        /**
         * @brief Publishes the current sub-phase of the maneuver.
         * @param phase The phase to report.
         */
        void set_phase(Phase phase)
        {
            if (this->manager_) {
                this->manager_->set_maneuver_phase(this->maneuver_, phase);
            }
        }

        /**
         * @brief Atomically changes the active maneuver (e.g. Mission to ReturnToLaunch).
         * @param next The maneuver to become active.
         * @return false if the scope no longer owns the active maneuver.
         */
        bool transition_to(ActiveManeuver next)
        {
            if (this->manager_ && this->manager_->transition_maneuver(this->maneuver_, next)) {
                this->maneuver_ = next;
                return true;
            }
            return false;
        }

    private:

        friend class FlightStateManager;

        /**
         * @brief Private constructor accessible only by FlightStateManager.
         * @param manager  Pointer to the owning FlightStateManager.
         * @param maneuver The maneuver being started.
         */
        ActionScope(FlightStateManager* manager, ActiveManeuver maneuver)
            : manager_(manager), 
              maneuver_(maneuver) {}

        FlightStateManager* manager_{nullptr};               ///< Pointer to owning state manager.
        ActiveManeuver      maneuver_{ActiveManeuver::None}; ///< Active maneuver tied to this scope.
        bool                succeeded_{false};               ///< Flag indicating normal completion.
    };

    /**
     * @brief Constructor for FlightStateManager.
     * @param flight_action_executor Reference to the flight action executor (composed module).
     * @param flight_primitives      Reference to the flight primitives (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param mission_executor       Reference to the mission executor (composed module).
     * @param ground_station_logger  Reference to the ground station logger (composed module).
     * @param manual_control_handler Reference to the manual control handler (composed module).
     * @param telemetry_publisher    Reference to the telemetry publisher (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     */
    FlightStateManager(
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives& flight_primitives,
        MissionHandler& mission_handler,
        MissionExecutor& mission_executor,
        GroundStationLogger& ground_station_logger,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher& telemetry_publisher,
        ManeuverRunner& maneuver_runner
    );

    /// @brief Destructor for FlightStateManager.
    ~FlightStateManager() = default;

    /**
     * @brief Atomically attempts to start an autonomous maneuver.
     * @param maneuver The maneuver to start.
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
     * @return true if the drone is armed and either no maneuver is active and Posctl was requested, 
     *         or the active maneuver is in Phase::Navigating.
     */
    bool should_virtual_stick_be_enabled(bool is_armed) const;

    /**
     * @brief Checks whether MANUAL_CONTROL inputs may be forwarded to the drone.
     * @return true only when no maneuver is active, so joystick frames cannot cancel navigation commands.
     */
    bool is_manual_input_allowed() const;

    /// @brief Returns the currently active maneuver.
    ActiveManeuver get_active_maneuver() const;

private:

    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    // FlightStateManager&   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    FlightActionExecutor& flight_action_executor_; ///< Entry points for takeoff, land, rtl, change altitude and go-to.
    GroundStationLogger&  ground_station_logger_;  ///< Logger utility for dispatching STATUSTEXT messages and alerts to the GCS.
    FlightPrimitives&     flight_primitives_;      ///< Blocking and cancellable flight operations.
    MissionHandler&       mission_handler_;        ///< Mission plan storage and progress reporter.
    MissionExecutor&      mission_executor_;       ///< Mission interpreter with pause and resume.
    ManualControlHandler& manual_control_handler_; ///< Handler for DJI Virtual Stick and MANUAL_CONTROL inputs.
    TelemetryPublisher&   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.


    // --------------------------
    // PRIVATE METHODS
    // --------------------------

    /**
     * @brief Internal callback invoked by ActionScope destructor to exit a maneuver.
     * @param maneuver  The maneuver being finished.
     * @param succeeded Whether the maneuver completed normally or aborted.
     */
    void finish_maneuver(ActiveManeuver maneuver, bool succeeded);

    /**
     * @brief Internal callback invoked by ActionScope to publish a phase.
     * @param maneuver The maneuver owning the scope.
     * @param phase    The new phase.
     */
    void set_maneuver_phase(ActiveManeuver maneuver, Phase phase);

    /**
     * @brief Internal callback invoked by ActionScope to change the active maneuver.
     * @param from Maneuver currently owned by the scope.
     * @param to   Maneuver to become active.
     * @return false if `from` is no longer the active maneuver.
     */
    bool transition_maneuver(ActiveManeuver from, ActiveManeuver to);


    // --------------------------
    // STATE
    // --------------------------

    mutable std::mutex mutex_;                                 ///< Mutex protecting all state variables.
    FlightMode         desired_mode_{FlightMode::Hold};        ///< Desired flight mode outside maneuvers.
    FlightMode         mode_before_maneuver_{FlightMode::Hold};///< Mode saved when the active maneuver began (restored by Reposition).
    ActiveManeuver     active_maneuver_{ActiveManeuver::None}; ///< Currently running maneuver.
    Phase              phase_{Phase::None};                    ///< Sub-phase of the running maneuver.
    bool               want_virtual_stick_{false};             ///< Flag indicating if current mode requires V-Stick.
    bool               override_during_maneuver_{false};       ///< Set when the pilot took over via RC while a maneuver was running.
};