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


    /** @brief Requests the drone to take off
     *  @param on_complete Optional callback function to be executed after the takeoff request is processed.
     */
    void request_takeoff(std::function<void()> on_complete = []() {});

    /** @brief Requests the drone to take off
     *  @param on_complete Optional callback function to be executed after the takeoff request is processed.
     */
    void request_arm(std::function<void()> on_complete = []() {});

    /** @brief Requests the drone to disarm
     *  @param on_complete Optional callback function to be executed after the disarm request is processed.
     */
    void request_disarm(std::function<void()> on_complete = []() {});


private:
    HttpHandler http_client_;
};