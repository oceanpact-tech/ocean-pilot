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
#include <boost/asio/io_context.hpp>

#include "wild_bridge_endpoints.hpp"
#include "http_handler.hpp"


/* Aliases  ------------------------------------------------------------------*/

namespace net = boost::asio;


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Main interface class for managing communication with the DJI WildBridge.
 * 
 * This class acts as the central hub for dispatching commands (like takeoff) 
 * and managing incoming telemetry data through the asynchronous event loop.
 */
class DJIInterface 
{
public:
    // Constructor
    DJIInterface(net::io_context& ioc, const std::string rc_ip)
    : rc_ip_(rc_ip), http_client_(ioc, rc_ip, "8080") 
    {}

    // TODO: calls this->stop_telemetry_stream()
    // // Destructor
    // ~DJIInterface();

    // TODO
    /* @brief Starts telemetry stream
     * @param none
     * @return none
     */
    // void start_telemetry_stream();

    // TODO
    /* @brief Stops telemetry stream
     * @param none
     * @return none
     */
    // void stop_telemetry_stream();


    void request_takeoff(std::function<void()> on_complete = nullptr);


private:
    std::string rc_ip_;  /** RC IP*/ 
    HttpHandler http_client_;
    

    // TODO
    // // Telemetry params
    // std::thread telemetry_thread_;
    // std::mutex telemetry_lock_;
    // boost::json::object latest_telemetry_;
    // bool telemetry_running_ = false;


    // TODO
    /* @brief Stops telemetry stream
     * @param none
     * @return none
     */
    // void telemetry_receiver_();
};