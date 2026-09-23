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
#include "telemetry_handler.hpp"


/* Aliases  ------------------------------------------------------------------*/

namespace net = boost::asio;


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Main interface class for managing communication with the DJI WildBridge.
 * 
 * Interface for DJI drone control via HTTP commands (port 8080) and 
 * TCP telemetry socket (port 8081).
 */
class DJIInterface 
{
public:

    // Constructor
    DJIInterface(net::io_context& ioc, const std::string host)
    : host_(host), 
      http_client_(ioc, host, "8080"),
      telemetry_client_(ioc, host)
    {}
    // TODO: Implementar o autodescobrir e iniciar o handle http posteriormente com o ip descoberto

    // Destructor
    ~DJIInterface();

    /** 
     * @brief Starts telemetry stream
     */
    void start_telemetry_stream();

    /** 
     * @brief Stops telemetry stream
     */
    void stop_telemetry_stream();

    /** @brief Requests the drone to take off
     *  @param on_complete Optional callback function to be executed after the takeoff request is processed.
     */
    void request_takeoff(std::function<void()> on_complete = nullptr);

    boost::json::object request_all_states(bool verbose = false) { return this->telemetry_client_.request_all_states(verbose); }
    Vector3D get_speed() { return this->telemetry_client_.get_speed(); }


private:
    std::string host_;  /// RC IP
    HttpHandler http_client_;
    TelemetryHandler telemetry_client_;
};