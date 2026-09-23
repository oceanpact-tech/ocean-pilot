/**
 ******************************************************************************
 * @file    telemetry_handler.cpp
 * @brief   Implementation of the TelemetryHandler class.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


/* Private Includes ----------------------------------------------------------*/

#include "telemetry_handler.hpp"


/* Public Class Methods ------------------------------------------------------*/

boost::json::object TelemetryHandler::get_telemetry() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    return this->latest_telemetry_;
}

boost::json::object TelemetryHandler::request_all_states(bool verbose) 
{
    auto telemetry = this->get_telemetry();
    if (verbose && !telemetry.empty()) {
        std::cout << "Telemetria:\n" << boost::json::serialize(telemetry) << "\n";
    }
    return telemetry;
}

std::string TelemetryHandler::get_drone_name() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("droneName") && telemetry.at("droneName").is_string()) {
        return boost::json::value_to<std::string>(telemetry.at("droneName"));
    }
    return ""; 
}

Vector3D TelemetryHandler::get_speed() 
{
    Vector3D speed; // Initialize with {x: 0.0, y:0.0, z:0.0}
    auto telemetry = this->get_telemetry();

    if (telemetry.contains("speed") && telemetry.at("speed").is_object()) 
    {
        const auto& speed_obj = telemetry.at("speed").as_object();

        auto extract_number = [](const boost::json::value& val) -> double {
            if (val.is_double()) return val.as_double();
            if (val.is_int64()) return static_cast<double>(val.as_int64());
            if (val.is_uint64()) return static_cast<double>(val.as_uint64());
            return 0.0; // Default if it is not a number
        };
        
        if (speed_obj.contains("x")) speed.x = extract_number(speed_obj.at("x"));
        if (speed_obj.contains("y")) speed.y = extract_number(speed_obj.at("y"));
        if (speed_obj.contains("z")) speed.z = extract_number(speed_obj.at("z"));
    }

    return speed; 
}


/* Protected Class Methods ---------------------------------------------------*/

void TelemetryHandler::process_raw_line(const std::string& line) 
{
    try 
    {
        // Try to convert the line to JSON
        boost::json::value jv = boost::json::parse(line);
        
        std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
        this->latest_telemetry_ = jv.as_object();
    } 
    catch (const std::exception& e) 
    {
        // Ignore malformed JSON lines, but log the error for debugging
    }
}