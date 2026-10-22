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
#include <cstdint>

// MAVSDK
#include <mavsdk/plugins/action_server/action_server.hpp>
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>

#include "dji_interface/dji_interface.hpp"
#include "mavlink_bridge/flight_state_manager.hpp"


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
     * @param state_manager         Reference to the flight state manager.
     * @param mavlink_direct_server Reference to the MAVSDK direct message plugin.
     */
    FlightActionExecutor(
        DJIInterface& dji,
        FlightStateManager& state_manager,
        mavsdk::MavlinkDirectServer& mavlink_direct_server
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
    void execute_change_altitude(float target_altitude_amsl);

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    DJIInterface&                dji_;                   ///< DJI interface for command dispatch and status polling.
    FlightStateManager&          state_manager_;         ///< State manager for acquiring ActionScope locks.
    mavsdk::MavlinkDirectServer& mavlink_direct_server_; ///< MAVSDK direct server for sending custom COMMAND_ACKs.

    // --------------------------
    // WORKER THREADS
    // --------------------------

    std::jthread takeoff_worker_;         ///< Cooperative worker thread for takeoff execution.
    std::jthread land_worker_;            ///< Cooperative worker thread for landing execution.
    std::jthread rtl_worker_;             ///< Cooperative worker thread for return to launch execution.
    std::jthread change_altitude_worker_; ///< Cooperative worker thread for change altitude execution.

    // --------------------------
    // FLAGS
    // --------------------------

    std::atomic<bool> is_changing_altitude_{false}; 

    // --------------------------
    // PRIVATE METHODS
    // --------------------------

    /**
     * @brief Dispatches a custom COMMAND_ACK message via MavlinkDirectServer.
     * @param command The MAV_CMD identifier being acknowledged.
     * @param result  The MAV_RESULT status code.
     */
    void send_command_ack(uint16_t command, uint8_t result);
};