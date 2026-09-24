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
    if (verbose && !telemetry.empty())
        std::cout << "Telemetria:\n" << boost::json::serialize(telemetry) << "\n";
    return telemetry;
}

std::string TelemetryHandler::get_drone_name() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("droneName") && telemetry.at("droneName").is_string())
        return boost::json::value_to<std::string>(telemetry.at("droneName"));
    return ""; 
}

Speed3D TelemetryHandler::get_speed() 
{
    Speed3D speed; // Initialize with {x: 0.0, y: 0.0, z: 0.0}
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("speed") && telemetry.at("speed").is_object()) 
    {
        const auto& speed_obj = telemetry.at("speed").as_object();
        if (speed_obj.contains("x")) speed.x = this->jv_to_double(speed_obj.at("x"));
        if (speed_obj.contains("y")) speed.y = this->jv_to_double(speed_obj.at("y"));
        if (speed_obj.contains("z")) speed.z = this->jv_to_double(speed_obj.at("z"));
    }
    return speed; 
}

double TelemetryHandler::get_heading() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("heading") && telemetry.at("heading").is_double())
        return boost::json::value_to<double>(telemetry.at("heading"));
    return 0.0; // Default if not available
}

Attitude3D TelemetryHandler::get_attitude() 
{
    Attitude3D attitude; // Initialize with {roll: 0.0, pitch: 0.0, yaw: 0.0}
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("attitude") && telemetry.at("attitude").is_object()) 
    {
        const auto& attitude_obj = telemetry.at("attitude").as_object();
        if (attitude_obj.contains("roll"))  attitude.roll  = this->jv_to_double(attitude_obj.at("roll"));
        if (attitude_obj.contains("pitch")) attitude.pitch = this->jv_to_double(attitude_obj.at("pitch"));
        if (attitude_obj.contains("yaw"))   attitude.yaw   = this->jv_to_double(attitude_obj.at("yaw"));
    }
    return attitude; 
}

Location3D TelemetryHandler::get_location() 
{
    Location3D location; // Initialize with {latitude: 0.0, longitude: 0.0, altitude: 0.0}
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("location") && telemetry.at("location").is_object()) 
    {
        const auto& location_obj = telemetry.at("location").as_object();
        if (location_obj.contains("latitude"))  location.latitude  = this->jv_to_double(location_obj.at("latitude"));
        if (location_obj.contains("longitude")) location.longitude = this->jv_to_double(location_obj.at("longitude"));
        if (location_obj.contains("altitude"))  location.altitude  = this->jv_to_double(location_obj.at("altitude"));
    }
    return location; 
}

Attitude3D TelemetryHandler::get_gimbal_attitude() 
{
    Attitude3D attitude; // Initialize with {roll: 0.0, pitch: 0.0, yaw: 0.0}
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("gimbalAttitude") && telemetry.at("gimbalAttitude").is_object()) 
    {
        const auto& attitude_obj = telemetry.at("gimbalAttitude").as_object();
        if (attitude_obj.contains("roll"))  attitude.roll  = this->jv_to_double(attitude_obj.at("roll"));
        if (attitude_obj.contains("pitch")) attitude.pitch = this->jv_to_double(attitude_obj.at("pitch"));
        if (attitude_obj.contains("yaw"))   attitude.yaw   = this->jv_to_double(attitude_obj.at("yaw"));
    }
    return attitude; 
}

Attitude3D TelemetryHandler::get_gimbal_joint_attitude() 
{
    Attitude3D attitude; // Initialize with {roll: 0.0, pitch: 0.0, yaw: 0.0}
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("gimbalJointAttitude") && telemetry.at("gimbalJointAttitude").is_object()) 
    {
        const auto& attitude_obj = telemetry.at("gimbalJointAttitude").as_object();
        if (attitude_obj.contains("roll"))  attitude.roll  = this->jv_to_double(attitude_obj.at("roll"));
        if (attitude_obj.contains("pitch")) attitude.pitch = this->jv_to_double(attitude_obj.at("pitch"));
        if (attitude_obj.contains("yaw"))   attitude.yaw   = this->jv_to_double(attitude_obj.at("yaw"));
    }
    return attitude; 
}

double TelemetryHandler::get_zoom_focal_length() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("zoomFl") && telemetry.at("zoomFl").is_double()) 
        return boost::json::value_to<double>(telemetry.at("zoomFl"));
    return 0.0; // Default if not available
}

double TelemetryHandler::get_hybrid_focal_length() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("hybridFl") && telemetry.at("hybridFl").is_double())
        return boost::json::value_to<double>(telemetry.at("hybridFl"));
    return 0.0; // Default if not available
}

double TelemetryHandler::get_optical_focal_length() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("opticalFl") && telemetry.at("opticalFl").is_double()) 
        return boost::json::value_to<double>(telemetry.at("opticalFl"));
    return 0.0; // Default if not available
}

double TelemetryHandler::get_zoom_ratio() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("zoomRatio") && telemetry.at("zoomRatio").is_double())
        return boost::json::value_to<double>(telemetry.at("zoomRatio"));
    return 0.0; // Default if not available
}

int TelemetryHandler::get_battery_level() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("batteryLevel")) 
    {
        const auto& battery_level = telemetry.at("batteryLevel");
        if (battery_level.is_int64())
            return static_cast<int>(battery_level.as_int64());
        if (battery_level.is_uint64())
            return static_cast<int>(battery_level.as_uint64());
    }
    return 0; // Default if not available
}

int TelemetryHandler::get_satellite_count() 
{
    auto telemetry = this->get_telemetry();
    if (telemetry.contains("satelliteCount")) 
    {
        const auto& satellite_count = telemetry.at("satelliteCount");
        if (satellite_count.is_int64())
            return static_cast<int>(satellite_count.as_int64());
        if (satellite_count.is_uint64())
            return static_cast<int>(satellite_count.as_uint64());
    }
    return 0; // Default if not available
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