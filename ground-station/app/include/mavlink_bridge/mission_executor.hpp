/**
 ******************************************************************************
 * @file    mission_executor.hpp
 * @brief   Interprets and executes a stored MAVLink mission item by item.
 *
 * This file defines the MissionExecutor class. A mission is a single 
 * ActiveManeuver::Mission, so the flight mode reported to the GCS stays 
 * "Mission" while internal steps (takeoff, waypoints, land, return to launch) 
 * run as FlightPrimitives without opening scopes of their own.
 *
 * Failure policy: a failed or timed out item triggers a return to launch 
 * inside the same scope (the reported mode becomes ReturnToLaunch). If the 
 * pilot takes over via RC, no command is issued. Pause only brakes the drone 
 * and keeps the progress so the mission can be resumed.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stop_token>

// DJI
#include "dji_interface/dji_interface.hpp"

// MAVSDK API
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.hpp>

// Composed modules
#include "mavlink_bridge/module_fwd.hpp"
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/flight_primitives.hpp"
#include "mavlink_bridge/mission_handler.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Mission interpreter with pause and resume support.
 * 
 * Reads a snapshot of the plan stored by MissionHandler and runs each item 
 * through FlightPrimitives. After every item, including those that only update 
 * context, it reports the item as reached so the MISSION_CURRENT sequence stays 
 * aligned with the plan.
 * 
 * Module references are stored for future extensions and must not be used 
 * during construction or destruction.
 */
class MissionExecutor 
{
public:

    /**
     * @brief Constructor for MissionExecutor.
     * @param dji                    Reference to the DJI hardware interface.
     * @param state_manager          Reference to the flight state manager (composed module).
     * @param flight_primitives      Reference to the flight primitives (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param flight_action_executor Reference to the flight action executor (composed module).
     * @param ground_station_logger  Reference to the ground station logger (composed module).
     * @param manual_control_handler Reference to the manual control handler (composed module).
     * @param telemetry_publisher    Reference to the telemetry publisher (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     */
    MissionExecutor(
        // DJI API
        DJIInterface& dji,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives&     flight_primitives,
        MissionHandler&       mission_handler,
        GroundStationLogger&  ground_station_logger,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher&   telemetry_publisher,
        ManeuverRunner&       maneuver_runner
    );

    /// @brief Destructor for MissionExecutor.
    ~MissionExecutor() = default;

    /**
     * @brief Starts the stored mission, or resumes it from the paused item.
     * 
     * Refuses (and reports the reason to the GCS) when there is no mission, 
     * when the pilot has manual control, or when the drone is on the ground 
     * and the plan does not begin with a takeoff. Returns false without 
     * reporting when the mission is already running (heartbeat echo).
     * 
     * @return true if the mission maneuver was started.
     */
    bool start();

    /// @brief Brakes the drone and keeps the progress, if a mission is running.
    void pause();

    /// @brief Discards any saved progress (new or cleared plan).
    void reset_progress();

    /// @brief Returns true while the mission maneuver is running.
    bool is_active() const;

    /// @brief Returns true if a paused mission can be resumed.
    bool is_paused() const;

private:

    // --------------------------
    // TYPES
    // --------------------------

    /// @brief Progress kept across pause and resume.
    struct Progress {
        bool        valid{false};    ///< True if there is something to resume.
        uint32_t    plan_version{0}; ///< Plan version the progress belongs to.
        std::size_t next_index{0};   ///< Next item to execute.
        float       speed_m_s{5.0f}; ///< Cruise speed set by DO_CHANGE_SPEED.
    };


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
    // MissionExecutor&      mission_executor_;       ///< Mission interpreter with pause and resume.
    GroundStationLogger&  ground_station_logger_;  ///< Logger utility for dispatching STATUSTEXT messages and alerts to the GCS.
    ManualControlHandler& manual_control_handler_; ///< Handler for DJI Virtual Stick and MANUAL_CONTROL inputs.
    TelemetryPublisher&   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.


    // --------------------------
    // STATE
    // --------------------------

    mutable std::mutex progress_mutex_;         ///< Protects progress_.
    Progress           progress_;               ///< Progress kept across pause and resume.
    std::atomic<bool>  pause_requested_{false}; ///< Distinguishes a pause from a preemption when the stop token fires.


    // --------------------------
    // PRIVATE METHODS
    // --------------------------

    /**
     * @brief Body of the mission maneuver (runs on the runner thread).
     * @param snapshot Plan snapshot being executed.
     * @param progress Starting progress.
     * @param stoken   Cooperative cancellation token.
     * @param scope    Maneuver scope owned by the runner.
     */
    void run_mission(
        const MissionHandler::PlanSnapshot& snapshot,
        Progress progress,
        std::stop_token stoken,
        FlightStateManager::ActionScope& scope
    );

    /**
     * @brief Executes a single mission item through the flight primitives.
     * @param item     Mission item.
     * @param progress Progress updated by context items (speed).
     * @param stoken   Cooperative cancellation token.
     * @param scope    Maneuver scope that receives the phase of each primitive.
     * @return The outcome of the item.
     */
    Outcome run_item(
        const mavsdk::MissionRawServer::MissionItem& item,
        Progress& progress,
        std::stop_token stoken,
        FlightStateManager::ActionScope& scope
    );

    /**
     * @brief Escalates to a return to launch after a failed item.
     * @param failed_item Index of the item that failed.
     * @param stoken      Cooperative cancellation token.
     * @param scope       Maneuver scope, transitioned to ReturnToLaunch.
     */
    void return_to_launch_after_failure(
        std::size_t failed_item,
        std::stop_token stoken,
        FlightStateManager::ActionScope& scope
    );

    /// @brief Saves progress so the mission can be resumed after a pause.
    void save_progress(const Progress& progress);
};