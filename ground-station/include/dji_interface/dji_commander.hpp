/**
 ******************************************************************************
 * @file    dji_interface.hpp
 * @brief   Interface for the DJI drone communication and control.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


#pragma once


/* Includes ------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <functional>

// Asio
// #include <boost/asio/io_context.hpp>

#include "transport/http_handler.hpp"
#include "dji_interface/wild_bridge_endpoints.hpp"


/* Aliases  ------------------------------------------------------------------*/

namespace net = boost::asio;


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Main interface class for managing communication with the DJI WildBridge.
 * 
 * Interface for DJI drone control via HTTP commands (port 8080) and 
 * TCP telemetry socket (port 8081).
 */
class DJICommander
{
public:

    /** @brief Constructor for DJICommander
     *  @param ioc 
     *  @param host 
     */
    DJICommander(
        net::io_context& ioc, 
        const std::string host, 
        const std::string port
    )
        : http_client_(ioc, host, port) {};

    /** @brief  Destructor for MavlinkBridge
     */
    ~DJICommander() = default;

    /**
     * @brief Requests the drone to take off.
     * @param on_complete Optional callback function to be executed after the request is processed.
     */
    void request_takeoff(std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Requests the drone to land.
     * @param on_complete Optional callback function to be executed after the request is processed.
     */
    void request_land(std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Requests the drone to return to home.
     * @param on_complete Optional callback function to be executed after the request is processed.
     */
    void request_return_to_home(std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Requests the drone to enable virtual stick.
     * @param on_complete Optional callback function to be executed after the request is processed.
     */
    void enable_virtual_stick(std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Requests the drone to disable virtual stick.
     * @param on_complete Optional callback function to be executed after the request is processed.
     */
    void disable_virtual_stick(std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Send virtual stick.
     * @param on_complete Optional callback function to be executed after the request is processed.
     * @param payload "leftX,leftY,rightX,rightY". Each stick value should be in the interval [-1,1].
     */
    void send_stick(std::string payload, std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Go to altitude.
     * @param on_complete Optional callback function to be executed after the request is processed.
     * @param altitude Relative altitude in meters.
     */
    void go_to_altitude(float altitude, std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Go to altitude by position.
     * @param on_complete Optional callback function to be executed after the request is processed.
     * @param altitude Relative altitude in meters.
     */
    void go_to_altitude_by_position(float altitude, std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Send DJI Native trajectory.
     * @param on_complete Optional callback function to be executed after the request is processed.
     * @param payload "speed;lat,lon,alt;...".
     */
    void send_dji_native_trajectory(std::string payload, std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Navigate to waypoint. Turn to face the leg, fly forward, rotate to yaw on arrival.
     * @param on_complete Optional callback function to be executed after the request is processed.
     * @param payload "lat,lon,alt,yaw[,speed]". yaw = final heading.
     */
    void go_to_waypoint_nose_forward(std::string payload, std::function<void(bool, std::string)> on_complete = nullptr);

    /**
     * @brief Navigate to waypoint. Hold yaw for the whole flight (drone crabs sideways), tighter arrival tolerance.
     * @param on_complete Optional callback function to be executed after the request is processed.
     * @param payload "lat,lon,alt,yaw[,speed]".
     */
    void go_to_waypoint_hold_heading(std::string payload, std::function<void(bool, std::string)> on_complete = nullptr);

private:

    HttpHandler http_client_; ///< HTTP client.
};