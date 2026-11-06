/**
 ******************************************************************************
 * @file    flight_primitives.hpp
 * @brief   Blocking and cancellable flight operations on top of the DJI WildBridge.
 *
 * This file defines the FlightPrimitives class, which owns every wait loop
 * used to command the aircraft (takeoff, land, return to launch, change 
 * altitude, go to waypoint, brake, hold). It knows nothing about MAVLink or 
 * ACKs: each operation blocks the calling thread until the drone reports 
 * completion, a timeout expires, the stop token fires or the pilot takes 
 * over via RC.
 *
 * Each operation publishes its own Phase in the ActionScope it receives. 
 * Navigation phases require the Virtual Stick to be enabled, while takeoff, 
 * land and return to launch require it disabled, so every operation waits 
 * for the Virtual Stick to reach the right state before commanding the drone.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <chrono>
#include <functional>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>

// DJI
#include "dji_interface/dji_interface.hpp"

// Composed modules
#include "mavlink_bridge/module_fwd.hpp"
#include "mavlink_bridge/flight_state_manager.hpp"


/* Enums  --------------------------------------------------------------------*/

/// @brief Result of a blocking flight operation.
enum class Outcome {
    Done,       ///< Operation completed successfully.
    Failed,     ///< Precondition not met, HTTP request rejected or operation never started.
    Timeout,    ///< Operation started but did not complete within its time budget.
    Cancelled,  ///< Stop token was triggered (pause, preemption or shutdown).
    Overridden  ///< Pilot took manual control via RC during the operation.
};

/// @brief Yaw policy used while flying to a waypoint.
enum class HeadingMode {
    NoseForward, ///< Turn to face the leg, fly forward, rotate to the final yaw on arrival.
    HoldHeading  ///< Keep the current yaw for the whole leg (drone crabs sideways).
};


/* Structs  ------------------------------------------------------------------*/

/// @brief Time budgets and polling period used by the wait loops.
struct PrimitiveTimeouts {
    std::chrono::seconds      start{10};             ///< Max wait for the drone to report the operation started.
    std::chrono::seconds      virtual_stick{10};     ///< Max wait for the Virtual Stick to reach the required state.
    std::chrono::seconds      takeoff{60};           ///< Max duration of a takeoff.
    std::chrono::seconds      land{120};             ///< Max duration of a landing.
    std::chrono::seconds      return_to_launch{600}; ///< Max duration of a return to launch.
    std::chrono::seconds      change_altitude{90};   ///< Max duration of an altitude change.
    std::chrono::seconds      waypoint{300};         ///< Max duration of a single waypoint attempt.
    std::chrono::seconds      brake{15};             ///< Max duration of a brake maneuver.
    std::chrono::milliseconds poll_interval{100};    ///< Polling period of drone status and position.
};

/// @brief Fully resolved navigation target (latitude and longitude in degrees, altitude relative to home).
struct WaypointCommand {
    Location3D           target;                                 ///< Latitude and longitude in degrees, altitude in meters relative to home.
    std::optional<float> yaw_deg;                                ///< Final heading in degrees; empty keeps the current heading.
    std::optional<float> speed_m_s;                              ///< Cruise speed in m/s; empty uses the app default.
    HeadingMode          heading_mode{HeadingMode::NoseForward}; ///< Yaw policy during the leg.
    int                  max_attempts{3};                        ///< Times the leg is re-sent when the waypoint is not confirmed as reached.
};


/* Functions  ----------------------------------------------------------------*/

/// @brief Converts an Outcome to a short text for GCS messages.
inline const char* to_string(Outcome outcome)
{
    switch (outcome) {
        case Outcome::Done:       return "done";
        case Outcome::Failed:     return "failed";
        case Outcome::Timeout:    return "timed out";
        case Outcome::Cancelled:  return "cancelled";
        case Outcome::Overridden: return "aborted by pilot";
    }
    return "unknown";
}


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Executor of blocking flight operations.
 * 
 * Every method is meant to run on the worker thread owned by ManeuverRunner. 
 * Methods poll the stop token, the pilot override and the armed state on every 
 * iteration, so cancellation latency is bounded by the polling period. An 
 * optional tick callback lets callers (e.g. the executor sending IN_PROGRESS 
 * ACKs) hook into the wait loops without this class knowing MAVLink.
 * 
 * Module references are stored for future extensions and must not be used 
 * during construction or destruction.
 */
class FlightPrimitives 
{
public:

    /// @brief Maneuver scope that receives the phase published by each operation.
    using Scope = FlightStateManager::ActionScope;

    /// @brief Callback invoked once per polling iteration while waiting.
    using TickFn = std::function<void()>;

    /**
     * @brief Constructor for FlightPrimitives.
     * @param dji                    Reference to the DJI hardware interface.
     * @param state_manager          Reference to the flight state manager (composed module).
     * @param flight_action_executor Reference to the flight action executor (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param mission_executor       Reference to the mission executor (composed module).
     * @param ground_station_logger  Reference to the ground station logger (composed module).
     * @param manual_control_handler Reference to the manual control handler (composed module).
     * @param telemetry_publisher    Reference to the telemetry publisher (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     * @param timeouts               Time budgets for each operation.
     */
    FlightPrimitives(
        // DJI API
        DJIInterface& dji,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        MissionHandler&       mission_handler,
        MissionExecutor&      mission_executor,
        GroundStationLogger&  ground_station_logger,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher&   telemetry_publisher,
        ManeuverRunner&       maneuver_runner,
        // General attributes
        PrimitiveTimeouts timeouts = {}
    );

    /// @brief Destructor for FlightPrimitives.
    ~FlightPrimitives() = default;

    /**
     * @brief Takes off and waits until the drone returns to IDLE.
     * @param scope   Maneuver scope that receives Phase::Takeoff.
     * @param stoken  Cooperative cancellation token.
     * @param on_tick Optional callback invoked while waiting.
     * @return Failed if the drone is not ready to take off or the request is rejected.
     */
    Outcome takeoff(Scope& scope, std::stop_token stoken, const TickFn& on_tick = {});

    /**
     * @brief Lands and waits until the drone is IDLE or disarmed.
     * @param scope   Maneuver scope that receives Phase::Landing.
     * @param stoken  Cooperative cancellation token.
     * @param on_tick Optional callback invoked while waiting.
     * @return Failed if the drone is not armed or the request is rejected.
     */
    Outcome land(Scope& scope, std::stop_token stoken, const TickFn& on_tick = {});

    /**
     * @brief Returns to launch and waits until the drone is IDLE or disarmed.
     * @param scope   Maneuver scope that receives Phase::ReturnToLaunch.
     * @param stoken  Cooperative cancellation token.
     * @param on_tick Optional callback invoked while waiting.
     * @return Failed if the drone is not armed or the request is rejected.
     */
    Outcome return_to_launch(Scope& scope, std::stop_token stoken, const TickFn& on_tick = {});

    /**
     * @brief Changes altitude and waits until the drone is hovering.
     * @param scope          Maneuver scope that receives Phase::Navigating.
     * @param altitude_rel_m Target altitude in meters, relative to home.
     * @param stoken         Cooperative cancellation token.
     * @param on_tick        Optional callback invoked while waiting.
     * @return Failed if the drone is not armed or the request is rejected.
     */
    Outcome change_altitude(Scope& scope, float altitude_rel_m, std::stop_token stoken, const TickFn& on_tick = {});

    /**
     * @brief Flies to a waypoint and confirms its arrival.
     * 
     * Sends the leg, waits for NAVIGATING and then IDLE, and confirms the 
     * arrival with the sequence id returned by the app. Unconfirmed legs are 
     * re-sent up to WaypointCommand::max_attempts times.
     * 
     * @param scope    Maneuver scope that receives Phase::Navigating.
     * @param waypoint Resolved navigation target.
     * @param stoken   Cooperative cancellation token.
     * @param on_tick  Optional callback invoked while waiting.
     * @return Failed if the drone is not armed, the request is rejected or the waypoint was never confirmed.
     */
    Outcome go_to_waypoint(Scope& scope, const WaypointCommand& waypoint, std::stop_token stoken, const TickFn& on_tick = {});

    /**
     * @brief Waits in place for a fixed duration (cancellable).
     * @param duration Time to wait.
     * @param stoken   Cooperative cancellation token.
     * @return Done when the duration elapsed.
     */
    Outcome hold(std::chrono::milliseconds duration, std::stop_token stoken);

    /**
     * @brief Stops the drone at its current position.
     * 
     * There is no dedicated hover endpoint, so this commands a go-to to the 
     * current position keeping the current heading. It is the cleanup of a 
     * cancelled maneuver, so it ignores the stop token (but obeys the pilot 
     * override) and must run while the scope is still alive.
     * 
     * @param scope   Maneuver scope that receives Phase::Navigating.
     * @param on_tick Optional callback invoked while waiting.
     * @return Done once the drone reports it is stopped.
     */
    Outcome brake(Scope& scope, const TickFn& on_tick = {});

private:

    // --------------------------
    // TYPES
    // --------------------------

    /// @brief Sends one HTTP request and reports its result through the given callback.
    using HttpSender = std::function<void(std::function<void(bool)>)>;


    // --------------------------
    // CORE REFERENCES
    // --------------------------

    DJIInterface&     dji_;      ///< DJI interface for telemetry polling and command dispatch.
    PrimitiveTimeouts timeouts_; ///< Time budgets for each operation.


    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    FlightStateManager&   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    FlightActionExecutor& flight_action_executor_; ///< Entry points for takeoff, land, rtl, change altitude and go-to.
    MissionHandler&       mission_handler_;        ///< Mission plan storage and progress reporter.
    MissionExecutor&      mission_executor_;       ///< Mission interpreter with pause and resume.
    GroundStationLogger&  ground_station_logger_;  ///< Logger utility for dispatching STATUSTEXT messages and alerts to the GCS.
    ManualControlHandler& manual_control_handler_; ///< Handler for DJI Virtual Stick and MANUAL_CONTROL inputs.
    TelemetryPublisher&   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.


    // --------------------------
    // PRIVATE METHODS
    // --------------------------

    /**
     * @brief Checks the conditions that must interrupt any operation.
     * @param stoken Cooperative cancellation token.
     * @return Overridden if the pilot took control, Cancelled if stop was requested, empty otherwise.
     */
    std::optional<Outcome> check_abort(const std::stop_token& stoken) const;

    /**
     * @brief Polls a predicate until it holds, a timeout expires or the operation is interrupted.
     * @param predicate Completion condition.
     * @param timeout   Time budget.
     * @param stoken    Cooperative cancellation token.
     * @param on_tick   Optional callback invoked once per iteration.
     * @return Done, Timeout, Cancelled or Overridden.
     */
    Outcome wait_until(
        const std::function<bool()>& predicate,
        std::chrono::seconds timeout,
        const std::stop_token& stoken,
        const TickFn& on_tick
    ) const;

    /**
     * @brief Waits until the Virtual Stick reaches the state required by the current phase.
     * @param enabled  Required Virtual Stick state.
     * @param stoken   Cooperative cancellation token.
     * @param on_tick  Optional callback invoked once per iteration.
     * @return Done once the state is reached, Timeout, Cancelled or Overridden otherwise.
     */
    Outcome wait_virtual_stick(bool enabled, const std::stop_token& stoken, const TickFn& on_tick) const;

    /**
     * @brief Sends a command and waits for the drone to report the expected status.
     * @param send           Function that dispatches the HTTP request.
     * @param started_status Drone status that confirms the operation started.
     * @param stoken         Cooperative cancellation token.
     * @param on_tick        Optional callback invoked once per iteration.
     * @return Done once started, Failed if the request was rejected or never started.
     */
    Outcome send_and_confirm_start(
        const HttpSender& send,
        std::string_view started_status,
        const std::stop_token& stoken,
        const TickFn& on_tick
    ) const;

    /**
     * @brief Dispatches the waypoint endpoint matching the requested heading mode.
     * @param waypoint    Resolved navigation target.
     * @param on_complete Callback with the HTTP result and the raw response body.
     */
    void send_waypoint(
        const WaypointCommand& waypoint,
        std::function<void(bool, std::string)> on_complete
    ) const;
};