#pragma once

#include <iostream>
#include <string>
#include <functional>

#include "wild_bridge_endpoints.hpp"
#include "http_handler.hpp"


class DJIInterface 
{
public:
    // Constructor
    DJIInterface(const std::string& rc_ip)
    : rc_ip_(rc_ip), http_client_(rc_ip, "8080") 
    {}

    // Destructor
    ~DJIInterface();

    /* @brief Starts telemetry stream
     * @param none
     * @return none
     */
    // void start_telemetry_stream();

    /* @brief Stops telemetry stream
     * @param none
     * @return none
     */
    // void stop_telemetry_stream();


    void request_takeoff(std::function<void()> on_complete = nullptr);


private:
    std::string rc_ip_;  /** RC IP*/ 
    HttpHandler http_client_;
    

    // // Telemetry params
    // std::thread telemetry_thread_;
    // std::mutex telemetry_lock_;
    // boost::json::object latest_telemetry_;
    // bool telemetry_running_ = false;


    /* @brief Stops telemetry stream
     * @param none
     * @return none
     */
    // void telemetry_receiver_();
};