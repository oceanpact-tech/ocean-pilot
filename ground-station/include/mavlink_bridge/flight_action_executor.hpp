/**
 ******************************************************************************
 * @file    flight_action_executor.hpp
 * @brief   Executes autonomous flight maneuvers (Arm, Takeoff, Land, Reposition).
 *
 * This file defines the FlightActionExecutor class, which manages the 
 * asynchronous execution of discrete flight commands requested by the Ground 
 * Control Station. It uses cooperative std::jthread workers and RAII state 
 * scopes to safely coordinate DJI HTTP requests and MAVLink COMMAND_ACK feedback.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <thread>
#include <vector>
#include <mutex>
#include <cstdint>

// MAVSDK
#include <mavsdk/plugins/action_server/action_server.hpp>
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.hpp>

#include "dji_interface/dji_interface.hpp"
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/ground_station_logger.hpp"
#include "mavlink_bridge/mission_handler.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Asynchronous executor for discrete flight commands and maneuvers.
 * 
 * The FlightActionExecutor handles Arm/Disarm validation and spawns managed 
 * background threads for long-running maneuvers such as Takeoff and Land. 
 * Upon destruction, all active maneuver threads are cooperatively stopped 
 * and joined to prevent dangling references.
 */
class FlightActionExecutor 
{
public:

    /**
     * @brief Constructor for FlightActionExecutor.
     * @param dji                   Reference to the DJI hardware interface.
     * @param state_manager         Reference to the flight state manager (composed module).
     * @param mavlink_direct_server Reference to the MAVSDK direct message plugin (mavsdk server).
     * @param ground_station_logger Reference to the ground station logger (composed module).
     * @param mission_handler       Reference to the mission handler (composed module).
     */
    FlightActionExecutor(
        DJIInterface& dji,
        FlightStateManager& state_manager,
        mavsdk::MavlinkDirectServer& mavlink_direct_server,
        mavsdk::ActionServer& action_server,
        GroundStationLogger& ground_station_logger,
        MissionHandler& mission_handler
    );

    /// @brief Destructor for FlightActionExecutor. Requests stop and joins worker threads.
    ~FlightActionExecutor();

    /**
     * @brief Handles incoming Arm/Disarm commands from the Ground Station.
     * @param command The ArmDisarm command structure received from ActionServer.
     */
    void handle_arm_disarm(const mavsdk::ActionServer::ArmDisarm& command);

    /// @brief Initiates the asynchronous takeoff sequence and dispatches COMMAND_ACKs.
    void execute_takeoff();

    /// @brief Initiates the asynchronous landing sequence and monitors touchdown.
    void execute_land();

    /// @brief Initiates the asynchronous returning sequence and monitors touchdown.
    void execute_return_to_launch();

    /**
     * @brief Initiates the asynchronous change altitude sequence and monitors touchdown..
     * @param target_altitude_amsl The altitude to be increased in meters.
     */
    void execute_change_altitude(const float& target_altitude_amsl);

    /**
     * @brief Initiates the asynchronous navigation to waypoint sequence.
     * @param coord Position3D object to store lat, lon and alt.
     */
    void execute_navigate_to_waypoint(const Location3D& waypoint);

    /// @brief Initiates the asynchronous mission sequence.
    void execute_mission();

    /**
     * @brief Save new mission plan coming from ground station.
     * @param plan Vector of MissionItem's.
     */
    inline void save_new_mission_plan(const mavsdk::MissionRawServer::MissionPlan& plan)
    {
        std::lock_guard<std::mutex> lock(this->mission_mutex_);
        this->latest_mission_plan_ = plan;
    }

    inline bool is_executing_mission() const 
    {
        return this->is_on_mission;
    }

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    // DJI Interface
    DJIInterface& dji_; ///< DJI interface for command dispatch and status polling.
    
    // MAVSDK
    mavsdk::MavlinkDirectServer& mavlink_direct_server_; ///< MAVSDK direct server for sending custom COMMAND_ACKs.
    mavsdk::ActionServer&        action_server_;         ///< MAVSDK action server.

    // --------------------------
    // COMPOSED MODULES
    // --------------------------
    
    FlightStateManager&  state_manager_;   ///< State manager for acquiring ActionScope locks.
    GroundStationLogger& gs_logger_;       ///< MAVSDK direct server for sending custom COMMAND_ACKs.
    MissionHandler&      mission_handler_; ///< Mission Handler; 

    // --------------------------
    // WORKER THREADS
    // --------------------------

    std::jthread takeoff_worker_;              ///< Cooperative worker thread for takeoff execution.
    std::jthread land_worker_;                 ///< Cooperative worker thread for landing execution.
    std::jthread rtl_worker_;                  ///< Cooperative worker thread for return to launch execution.
    std::jthread change_altitude_worker_;      ///< Cooperative worker thread for change altitude execution.
    std::jthread navigate_to_waypoint_worker_; ///< Cooperative worker thread fornavigate to waypoint execution.
    std::jthread mission_worker_;              ///< Cooperative worker thread for mission execution.

    // --------------------------
    // FLAGS
    // --------------------------

    std::atomic<bool> is_changing_altitude_{false}; 
    std::atomic<bool> is_navigating_to_waypoint_{false}; 
    std::atomic<bool> is_on_mission{false};

    // --------------------------
    // BUFFERS
    // --------------------------

    mavsdk::MissionRawServer::MissionPlan latest_mission_plan_;
    mutable std::mutex mission_mutex_;

    // --------------------------
    // PRIVATE METHODS
    // --------------------------

    /**
     * @brief Dispatches a custom COMMAND_ACK message via MavlinkDirectServer.
     * @param command The MAV_CMD identifier being acknowledged.
     * @param result  The MAV_RESULT status code.
     */
    void send_command_ack(uint16_t command, uint8_t result);

    inline mavsdk::MissionRawServer::MissionPlan get_latest_mission_plan() const
    {
        std::lock_guard<std::mutex> lock(this->mission_mutex_);
        return this->latest_mission_plan_;
    }
};