/**
 ******************************************************************************
 * @file    flight_action_executor.hpp
 * @brief   Entry points for flight commands requested by the Ground Station.
 *
 * This file defines the FlightActionExecutor class. Each command validates its 
 * preconditions, sends the COMMAND_ACK feedback and hands a body to the 
 * ManeuverRunner, which owns the worker thread and the state scope. The wait 
 * loops live in FlightPrimitives and the mission lives in MissionExecutor.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <cstdint>

// DJI
#include "dji_interface/dji_interface.hpp"

// MAVSDK API
#include <mavsdk/plugins/action_server/action_server.hpp>
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>

// Composed modules
#include "mavlink_bridge/module_fwd.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Command front-end for discrete flight maneuvers.
 * 
 * Land and return to launch preempt any running maneuver (including a mission). 
 * Takeoff and reposition are refused while another maneuver is running.
 * 
 * Module references are stored for future extensions and must not be used 
 * during construction or destruction.
 */
class FlightActionExecutor 
{
public:

    /**
     * @brief Constructor for FlightActionExecutor.
     * @param dji                    Reference to the DJI hardware interface.
     * @param mavlink_direct_server  Reference to the MAVSDK direct message plugin (mavsdk server).
     * 
     * @param state_manager          Reference to the flight state manager (composed module).
     * @param flight_primitives      Reference to the flight primitives (composed module).
     * @param mission_handler        Reference to the mission handler (composed module).
     * @param mission_executor       Reference to the mission executor (composed module).
     * @param ground_station_logger  Reference to the ground station logger (composed module).
     * @param manual_control_handler Reference to the manual control handler (composed module).
     * @param telemetry_publisher    Reference to the telemetry publisher (composed module).
     * @param maneuver_runner        Reference to the maneuver runner (composed module).
     */
    FlightActionExecutor(
        // DJI API
        DJIInterface& dji,
        // MAVSDK API
        mavsdk::MavlinkDirectServer& mavlink_direct_server,
        // Composed modules
        FlightStateManager&   state_manager,
        FlightPrimitives&     flight_primitives,
        MissionHandler&       mission_handler,
        MissionExecutor&      mission_executor,
        GroundStationLogger&  ground_station_logger,
        ManualControlHandler& manual_control_handler,
        TelemetryPublisher&   telemetry_publisher,
        ManeuverRunner&       maneuver_runner
    );

    /// @brief Destructor for FlightActionExecutor.
    ~FlightActionExecutor() = default;

    /**
     * @brief Handles incoming Arm/Disarm commands from the Ground Station.
     * @param command The ArmDisarm command structure received from ActionServer.
     */
    void handle_arm_disarm(const mavsdk::ActionServer::ArmDisarm& command);

    /// @brief Initiates the asynchronous takeoff sequence and dispatches COMMAND_ACKs.
    void execute_takeoff();

    /// @brief Initiates the asynchronous landing sequence, preempting any running maneuver.
    void execute_land();

    /// @brief Initiates the asynchronous returning sequence, preempting any running maneuver.
    void execute_return_to_launch();

    /**
     * @brief Initiates the asynchronous change altitude sequence (MAV_CMD_DO_REPOSITION).
     * @param target_altitude_amsl Target altitude in meters AMSL.
     */
    void execute_change_altitude(float target_altitude_amsl);

    /**
     * @brief Initiates the asynchronous navigation to waypoint sequence (MAV_CMD_DO_REPOSITION with coordinates).
     * @param waypoint Target latitude and longitude in degrees, and altitude in meters relative to home.
     */
    void execute_navigate_to_waypoint(const Location3D& waypoint);

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    DJIInterface&                dji_;                   ///< DJI interface for telemetry polling and command dispatch.
    mavsdk::MavlinkDirectServer& mavlink_direct_server_; ///< MAVSDK direct server for dispatching custom messages.


    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    FlightStateManager&   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    FlightPrimitives&     flight_primitives_;      ///< Blocking and cancellable flight operations.
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
     * @brief Dispatches a custom COMMAND_ACK message via MavlinkDirectServer.
     * @param command The MAV_CMD identifier being acknowledged.
     * @param result  The MAV_RESULT status code.
     */
    void send_command_ack(uint16_t command, uint8_t result);
};