/**
 ******************************************************************************
 * @file    maneuver_runner.hpp
 * @brief   Single worker thread that executes autonomous maneuvers.
 *
 * This file defines the ManeuverRunner class, the only owner of a maneuver 
 * thread. It opens the ActionScope in FlightStateManager, applies the 
 * preemption policy and guarantees cooperative stop and join.
 *
 * Lifetime rule: the runner is declared LAST among the composed modules, so 
 * it is destroyed first and joins the worker while every other module is still 
 * alive. MavlinkBridge also calls cancel() explicitly in its destructor.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <functional>
#include <mutex>
#include <stop_token>
#include <thread>

// Composed modules
#include "mavlink_bridge/module_fwd.hpp"
#include "mavlink_bridge/flight_state_manager.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Owner of the single maneuver worker thread.
 * 
 * Only one maneuver can run at a time. A body must never call run() or 
 * cancel() on its own runner, otherwise the join would wait for itself.
 * 
 * Module references are stored for future extensions and must not be used 
 * during construction or destruction.
 */
class ManeuverRunner 
{
public:

    using Maneuver = FlightStateManager::ActiveManeuver;

    /// @brief Body of a maneuver. Must be copy-constructible. The scope is owned by the runner.
    using Body = std::function<void(std::stop_token, FlightStateManager::ActionScope&)>;

    /// @brief What to do when a maneuver is requested while another one is running.
    enum class Policy {
        RejectIfBusy,  ///< Refuse the new maneuver.
        PreemptCurrent ///< Cancel and join the running maneuver, then start the new one.
    };

    /**
     * @brief Constructor for ManeuverRunner.
     * @param state_manager          Reference to the flight state manager (composed module).
     * @param flight_action_executor Reference to the flight action executor (composed module).
     * @param flight_primitives      Reference to the flight primitives (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param mission_executor       Reference to the mission executor (composed module).
     * @param ground_station_logger  Reference to the ground station logger (composed module).
     * @param manual_control_handler Reference to the manual control handler (composed module).
     * @param telemetry_publisher    Reference to the telemetry publisher (composed module).
     */
    ManeuverRunner(
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives&     flight_primitives,
        MissionHandler&       mission_handler,
        MissionExecutor&      mission_executor,
        GroundStationLogger&  ground_station_logger,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher&   telemetry_publisher
    );

    /// @brief Destructor for ManeuverRunner. Requests stop and joins the worker.
    ~ManeuverRunner();

    /**
     * @brief Starts a maneuver on the worker thread.
     * @param maneuver Maneuver to acquire in FlightStateManager.
     * @param policy   Behavior when another maneuver is running.
     * @param body     Code executed on the worker thread.
     * @return false if the maneuver was refused.
     */
    bool run(Maneuver maneuver, Policy policy, Body body);

    /// @brief Requests stop and joins the worker. Blocks until the running maneuver unwinds.
    void cancel();

    /// @brief Returns true while a maneuver holds the state manager slot.
    bool is_busy() const;

private:

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
    TelemetryPublisher&   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    // ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.
    

    // --------------------------
    // WORKER THREAD
    // --------------------------

    std::mutex   mutex_;  ///< Serializes run() and cancel() across callback threads.
    std::jthread worker_; ///< The single maneuver worker thread.
};