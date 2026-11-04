/**
 ******************************************************************************
 * @file    mission_handler.hpp
 * @brief   Stores MAVLink waypoint missions received from Ground Control.
 *
 * This file defines the MissionHandler class, responsible for receiving mission 
 * plans from QGroundControl via the MAVSDK MissionRawServer plugin, keeping the 
 * latest valid plan under a version number and reporting execution progress 
 * (MISSION_CURRENT / MISSION_ITEM_REACHED). It does not execute anything.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

// MAVSDK API
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.hpp>

// Composed modules
#include "mavlink_bridge/ground_station_logger.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Plan storage and mission progress reporter.
 * 
 * Only items of mission type MISSION are kept (fence and rally items are 
 * dropped). Every accepted upload or clear increments the plan version, 
 * so a paused mission can detect that its plan changed. 
 */
class MissionHandler 
{
public:

    using MissionPlan           = mavsdk::MissionRawServer::MissionPlan;
    using MissionItem           = mavsdk::MissionRawServer::MissionItem;
    using IncomingMissionHandle = mavsdk::MissionRawServer::IncomingMissionHandle;
    using ClearAllHandle        = mavsdk::MissionRawServer::ClearAllHandle;
    using Result                = mavsdk::MissionRawServer::Result;

    /// @brief Callback invoked (outside any lock) when the stored plan changes or is cleared.
    using PlanChangedCallback = std::function<void()>;

    /// @brief Immutable copy of the stored plan.
    struct PlanSnapshot {
        MissionPlan plan;            ///< Mission items.
        uint32_t    version{0};      ///< Version of the plan when the snapshot was taken.
    };

    /**
     * @brief Constructor for MissionHandler.
     * @param mission_server Reference to the MAVSDK mission raw server plugin.
     * @param gs_logger      Reference to the ground station logger (composed module).
     */
    MissionHandler(
        // MAVSDK API
        mavsdk::MissionRawServer& mission_server,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightActionExecutor& flight_action_executor,
        FlightPrimitives&     flight_primitives,
        MissionExecutor&      mission_executor,
        GroundStationLogger&  ground_station_logger,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher&   telemetry_publisher,
        ManeuverRunner&       maneuver_runner
    );

    /// @brief Destructor for MissionHandler. Unsubscribes from mission events.
    ~MissionHandler();

    /// @brief Starts listening for incoming and cleared missions.
    void start();

    /**
     * @brief Registers the callback notified when the plan changes.
     * @param callback Callback to invoke.
     */
    void set_plan_changed_callback(PlanChangedCallback callback);

    /**
     * @brief Returns a copy of the stored plan.
     * @return The snapshot, or std::nullopt if there is no mission.
     */
    std::optional<PlanSnapshot> get_plan() const;

    /// @brief Returns true if a mission is stored.
    bool has_mission() const;

    /**
     * @brief Reports the current item as reached and advances MISSION_CURRENT.
     * 
     * Not thread-safe against itself: only the mission worker should call it.
     */
    inline void report_item_reached()
    {
        this->mission_server_.set_current_item_complete();
    }

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    mavsdk::MissionRawServer& mission_server_; ///< MAVSDK raw mission server handling the MAVLink protocol.

    
    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    FlightStateManager&   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    FlightActionExecutor& flight_action_executor_; ///< Entry points for takeoff, land, rtl, change altitude and go-to.
    FlightPrimitives&     flight_primitives_;      ///< Blocking and cancellable flight operations.
    // MissionHandler&       mission_handler_;        ///< Mission plan storage and progress reporter.
    MissionExecutor&      mission_executor_;       ///< Mission interpreter with pause and resume.
    GroundStationLogger&  ground_station_logger_;  ///< Logger utility for dispatching STATUSTEXT messages and alerts to the GCS.
    ManualControlHandler& manual_control_handler_; ///< Handler for DJI Virtual Stick and MANUAL_CONTROL inputs.
    TelemetryPublisher&   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    ManeuverRunner&       maneuver_runner_;        ///< Owner of the single maneuver thread.


    // --------------------------
    // STORED PLAN
    // --------------------------

    mutable std::mutex mutex_;          ///< Protects plan_ and version_.
    std::optional<MissionPlan> plan_;   ///< Latest valid plan.
    uint32_t           version_{0};     ///< Incremented on every upload or clear.
    PlanChangedCallback on_plan_changed_; ///< Notified when the plan changes.


    // --------------------------
    // SUBSCRIPTION HANDLES
    // --------------------------

    IncomingMissionHandle incoming_mission_handle_{}; ///< Handle for incoming mission callback.
    ClearAllHandle        clear_all_handle_{};        ///< Handle for clear-all callback.


    // --------------------------
    // HANDLES CALLBACK
    // --------------------------

    /**
     * @brief Filters, validates and stores a mission received from the GCS.
     * @param result Transfer result.
     * @param plan   Received plan.
     */
    void on_incoming_mission(Result result, MissionPlan plan);

    /**
     * @brief Drops the stored plan when the GCS clears the mission.
     * @param mission_type The MAV_MISSION_TYPE that was cleared.
     */
    void on_clear_all(uint32_t mission_type);
};