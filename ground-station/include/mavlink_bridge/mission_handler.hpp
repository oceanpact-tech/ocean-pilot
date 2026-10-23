/**
 ******************************************************************************
 * @file    mission_handler.hpp
 * @brief   Handles incoming MAVLink waypoint missions from Ground Control.
 *
 * This file defines the MissionHandler class, responsible for intercepting 
 * mission plans sent by QGroundControl via the MAVSDK MissionRawServer plugin, 
 * parsing the raw MAVLink items, and formatting them for the DJI Android endpoint.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include <string>

// MAVSDK
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.hpp>

#include "dji_interface/dji_interface.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Controller for receiving and processing MAVLink missions.
 * 
 * The MissionHandler subscribes to the MAVSDK MissionRawServer plugin to 
 * automatically negotiate mission uploads from QGC. Upon success, it 
 * extracts the coordinates (command 16) and speeds (command 178), building 
 * a native trajectory payload that is forwarded to the drone via HTTP.
 */
class MissionHandler 
{
public:

    /**
     * @brief Constructor for MissionHandler.
     * @param dji            Reference to the DJI hardware interface.
     * @param mission_server Reference to the MAVSDK mission raw server plugin.
     */
    MissionHandler(DJIInterface& dji, mavsdk::MissionRawServer& mission_server);

    /// @brief Destructor for MissionHandler. Unsubscribes from mission events.
    ~MissionHandler();

    /// @brief Starts listening for incoming missions from the Ground Station.
    void start();

private:

    // --------------------------
    // CORE REFERENCES
    // --------------------------

    DJIInterface&             dji_;            ///< DJI interface for dispatching the mission payload.
    mavsdk::MissionRawServer& mission_server_; ///< MAVSDK raw mission server handling the MAVLink protocol.


    // --------------------------
    // SUBSCRIPTION HANDLES
    // --------------------------

    mavsdk::MissionRawServer::IncomingMissionHandle incoming_mission_handle_{}; ///< Handle for incoming mission callback.
};