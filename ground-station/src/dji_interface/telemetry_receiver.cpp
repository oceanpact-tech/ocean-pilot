/**
 ******************************************************************************
 * @file    telemetry_receiver.cpp
 * @brief   Implementation of the TelemetryReceiver class.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


/* Private Includes ----------------------------------------------------------*/

#include "dji_interface/telemetry_receiver.hpp"


/* Public Class Methods ------------------------------------------------------*/

boost::json::object TelemetryReceiver::get_telemetry() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    return this->latest_telemetry_;
}

bool TelemetryReceiver::is_telemetry_ready() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    return !this->latest_telemetry_.empty();
}

boost::json::object TelemetryReceiver::request_all_states(bool verbose) 
{
    auto telemetry = this->get_telemetry();
    if (verbose && !telemetry.empty())
        std::cout << "Telemetria:\n" << boost::json::serialize(telemetry) << "\n";
    return telemetry;
}

std::string TelemetryReceiver::get_drone_name() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("droneName") && telemetry.at("droneName").is_string())
        return boost::json::value_to<std::string>(telemetry.at("droneName"));
    return ""; 
}

Speed3D TelemetryReceiver::get_speed() 
{
    Speed3D speed; // Initialize with {x: 0.0, y: 0.0, z: 0.0}
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("speed") && telemetry.at("speed").is_object()) 
    {
        const auto& speed_obj = telemetry.at("speed").as_object();
        if (speed_obj.contains("x")) speed.x = this->jv_to_double(speed_obj.at("x"));
        if (speed_obj.contains("y")) speed.y = this->jv_to_double(speed_obj.at("y"));
        if (speed_obj.contains("z")) speed.z = this->jv_to_double(speed_obj.at("z"));
    }
    return speed; 
}

double TelemetryReceiver::get_heading() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("heading") && telemetry.at("heading").is_double())
        return boost::json::value_to<double>(telemetry.at("heading"));
    return 0.0; // Default if not available
}

Attitude3D TelemetryReceiver::get_attitude() 
{
    Attitude3D attitude; // Initialize with {roll: 0.0, pitch: 0.0, yaw: 0.0}
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("attitude") && telemetry.at("attitude").is_object()) 
    {
        const auto& attitude_obj = telemetry.at("attitude").as_object();
        if (attitude_obj.contains("roll"))  attitude.roll  = this->jv_to_double(attitude_obj.at("roll"));
        if (attitude_obj.contains("pitch")) attitude.pitch = this->jv_to_double(attitude_obj.at("pitch"));
        if (attitude_obj.contains("yaw"))   attitude.yaw   = this->jv_to_double(attitude_obj.at("yaw"));
    }
    return attitude; 
}

Location3D TelemetryReceiver::get_location() 
{
    Location3D location; // Initialize with {latitude: 0.0, longitude: 0.0, altitude: 0.0}
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("location") && telemetry.at("location").is_object()) 
    {
        const auto& location_obj = telemetry.at("location").as_object();
        if (location_obj.contains("latitude"))  location.latitude  = this->jv_to_double(location_obj.at("latitude"));
        if (location_obj.contains("longitude")) location.longitude = this->jv_to_double(location_obj.at("longitude"));
        if (location_obj.contains("altitude"))  location.altitude  = this->jv_to_double(location_obj.at("altitude"));
    }
    return location; 
}

Attitude3D TelemetryReceiver::get_gimbal_attitude() 
{
    Attitude3D attitude; // Initialize with {roll: 0.0, pitch: 0.0, yaw: 0.0}
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("gimbalAttitude") && telemetry.at("gimbalAttitude").is_object()) 
    {
        const auto& attitude_obj = telemetry.at("gimbalAttitude").as_object();
        if (attitude_obj.contains("roll"))  attitude.roll  = this->jv_to_double(attitude_obj.at("roll"));
        if (attitude_obj.contains("pitch")) attitude.pitch = this->jv_to_double(attitude_obj.at("pitch"));
        if (attitude_obj.contains("yaw"))   attitude.yaw   = this->jv_to_double(attitude_obj.at("yaw"));
    }
    return attitude; 
}

Attitude3D TelemetryReceiver::get_gimbal_joint_attitude() 
{
    Attitude3D attitude; // Initialize with {roll: 0.0, pitch: 0.0, yaw: 0.0}
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("gimbalJointAttitude") && telemetry.at("gimbalJointAttitude").is_object()) 
    {
        const auto& attitude_obj = telemetry.at("gimbalJointAttitude").as_object();
        if (attitude_obj.contains("roll"))  attitude.roll  = this->jv_to_double(attitude_obj.at("roll"));
        if (attitude_obj.contains("pitch")) attitude.pitch = this->jv_to_double(attitude_obj.at("pitch"));
        if (attitude_obj.contains("yaw"))   attitude.yaw   = this->jv_to_double(attitude_obj.at("yaw"));
    }
    return attitude; 
}

double TelemetryReceiver::get_zoom_focal_length() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("zoomFl"))
    {
        const auto& zoomFl = telemetry.at("zoomFl");
        if (zoomFl.is_double())
            return boost::json::value_to<double>(zoomFl);
        else if (zoomFl.is_int64())
            return static_cast<double>(zoomFl.as_int64());
        else if (zoomFl.is_uint64())
            return static_cast<double>(zoomFl.as_uint64());
        }
    return -1.0; // Default if not available
}

double TelemetryReceiver::get_hybrid_focal_length() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("hybridFl"))
    {
        const auto& hybridFl = telemetry.at("hybridFl");
        if (hybridFl.is_double())
            return boost::json::value_to<double>(hybridFl);
        else if (hybridFl.is_int64())
            return static_cast<double>(hybridFl.as_int64());
        else if (hybridFl.is_uint64())
            return static_cast<double>(hybridFl.as_uint64());
        }
    return -1.0; // Default if not available
}

double TelemetryReceiver::get_optical_focal_length() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("hybridFl"))
    {
        const auto& opticalFl = telemetry.at("opticalFl");
        if (opticalFl.is_double())
            return boost::json::value_to<double>(opticalFl);
        else if (opticalFl.is_int64())
            return static_cast<double>(opticalFl.as_int64());
        else if (opticalFl.is_uint64())
            return static_cast<double>(opticalFl.as_uint64());
        }
    return -1.0; // Default if not available
}

double TelemetryReceiver::get_zoom_ratio() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("zoomRatio") && telemetry.at("zoomRatio").is_double())
        return boost::json::value_to<double>(telemetry.at("zoomRatio"));
    return 0.0; // Default if not available
}

int TelemetryReceiver::get_battery_level() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
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

int TelemetryReceiver::get_satellite_count() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
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

bool TelemetryReceiver::is_armed() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("isArmed") && telemetry.at("isArmed").is_bool())
        return telemetry.at("isArmed").as_bool();
    return false; // Default if not available
}

bool TelemetryReceiver::is_ready_to_takeoff() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("readyToTakeoff") && telemetry.at("readyToTakeoff").is_bool())
        return telemetry.at("readyToTakeoff").as_bool();
    return false; // Default if not available
}

std::string TelemetryReceiver::get_takeoff_block_reason() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("takeoffBlockReason") && telemetry.at("takeoffBlockReason").is_string())
        return boost::json::value_to<std::string>(telemetry.at("takeoffBlockReason"));
    return ""; 
}

Location2D TelemetryReceiver::get_home_location() 
{
    Location2D location; // Initialize with {latitude: 0.0, longitude: 0.0}
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("homeLocation") && telemetry.at("homeLocation").is_object()) 
    {
        const auto& location_obj = telemetry.at("homeLocation").as_object();
        if (location_obj.contains("latitude"))  location.latitude  = this->jv_to_double(location_obj.at("latitude"));
        if (location_obj.contains("longitude")) location.longitude = this->jv_to_double(location_obj.at("longitude"));
    }
    return location; 
}

double TelemetryReceiver::get_home_altitude() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("homeAltitude") && telemetry.at("homeAltitude").is_double())
        return boost::json::value_to<double>(telemetry.at("homeAltitude"));
    return 0.0; // Default if not available
}

double TelemetryReceiver::get_distance_to_home() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("distanceToHome") && telemetry.at("distanceToHome").is_double())
        return boost::json::value_to<double>(telemetry.at("distanceToHome"));
    return 0.0; // Default if not available
}

Location3D TelemetryReceiver::get_lrf_target() 
{
    Location3D location; // Initialize with {latitude: 0.0, longitude: 0.0, altitude: 0.0}
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("lrfTarget")) 
    {
        if (telemetry.at("lrfTarget").is_object())
        {
            const auto& lrf_obj = telemetry.at("lrfTarget").as_object();
            if (lrf_obj.contains("latitude"))  location.latitude  = this->jv_to_double(lrf_obj.at("latitude"));
            if (lrf_obj.contains("longitude")) location.longitude = this->jv_to_double(lrf_obj.at("longitude"));
            if (lrf_obj.contains("altitude"))  location.altitude  = this->jv_to_double(lrf_obj.at("altitude"));
            return location;
        }
        location.latitude  = -1.0;
        location.longitude = -1.0;
        location.altitude  = -1.0;
    }
    return location; 
}

int TelemetryReceiver::get_current_target_waypoint_index()
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("currentTargetWaypointIndex")) 
    {
        const auto& waypoint_index = telemetry.at("currentTargetWaypointIndex");
        if (waypoint_index.is_int64())
            return static_cast<int>(waypoint_index.as_int64());
        if (waypoint_index.is_uint64())
            return static_cast<int>(waypoint_index.as_uint64());
    }
    return -1; // Default if not available 
}

int TelemetryReceiver::get_waypoint_seq() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("waypointSeq")) 
    {
        const auto& waypoint_seq = telemetry.at("waypointSeq");
        if (waypoint_seq.is_int64())
            return static_cast<int>(waypoint_seq.as_int64());
        if (waypoint_seq.is_uint64())
            return static_cast<int>(waypoint_seq.as_uint64());
    }
    return -1; // Default if not available
}

bool TelemetryReceiver::is_waypoint_reached(int seq) 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;

    bool reached = false;
    if (telemetry.contains("waypointReached") && telemetry.at("waypointReached").is_bool())
        reached = telemetry.at("waypointReached").as_bool();
    
    int waypoint_seq = -1;
    if (telemetry.contains("waypointSeq")) 
    {
        const auto& seq_val = telemetry.at("waypointSeq");
        if      (seq_val.is_int64())  waypoint_seq = static_cast<int>(seq_val.as_int64());
        else if (seq_val.is_uint64()) waypoint_seq = static_cast<int>(seq_val.as_uint64());
    }

    return reached && (waypoint_seq == seq);
}

bool TelemetryReceiver::is_intermediary_waypoint_reached() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("intermediaryWaypointReached") && telemetry.at("intermediaryWaypointReached").is_bool())
        return telemetry.at("intermediaryWaypointReached").as_bool();
    return false; // Default if not available
}

int TelemetryReceiver::get_yaw_seq() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("yawSeq")) 
    {
        const auto& yaw_seq = telemetry.at("yawSeq");
        if (yaw_seq.is_int64())
            return static_cast<int>(yaw_seq.as_int64());
        if (yaw_seq.is_uint64())
            return static_cast<int>(yaw_seq.as_uint64());
    }
    return -1; // Default if not available
}

bool TelemetryReceiver::is_yaw_reached(int seq) 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    bool reached = false;
    if (telemetry.contains("yawReached") && telemetry.at("yawReached").is_bool())
        reached = telemetry.at("yawReached").as_bool();
    
    int yaw_seq = -1;
    if (telemetry.contains("yawSeq")) 
    {
        const auto& seq_val = telemetry.at("yawSeq");
        if (seq_val.is_int64()) yaw_seq = static_cast<int>(seq_val.as_int64());
        else if (seq_val.is_uint64()) yaw_seq = static_cast<int>(seq_val.as_uint64());
    }

    return reached && (yaw_seq == seq);
}

int TelemetryReceiver::get_altitude_seq() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("altitudeSeq")) 
    {
        const auto& altitude_seq = telemetry.at("altitudeSeq");
        if (altitude_seq.is_int64())
            return static_cast<int>(altitude_seq.as_int64());
        if (altitude_seq.is_uint64())
            return static_cast<int>(altitude_seq.as_uint64());
    }
    return -1; // Default if not available
}

bool TelemetryReceiver::is_altitude_reached(int seq) 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    bool reached = false;
    if (telemetry.contains("altitudeReached") && telemetry.at("altitudeReached").is_bool())
        reached = telemetry.at("altitudeReached").as_bool();
    
    int altitude_seq = -1;
    if (telemetry.contains("altitudeSeq")) 
    {
        const auto& seq_val = telemetry.at("altitudeSeq");
        if (seq_val.is_int64()) altitude_seq = static_cast<int>(seq_val.as_int64());
        else if (seq_val.is_uint64()) altitude_seq = static_cast<int>(seq_val.as_uint64());
    }

    return reached && (altitude_seq == seq);
}

bool TelemetryReceiver::is_camera_recording() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("isRecording") && telemetry.at("isRecording").is_bool())
        return telemetry.at("isRecording").as_bool();
    return false; // Default if not available
}

bool TelemetryReceiver::is_home_set() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("homeSet") && telemetry.at("homeSet").is_bool())
        return telemetry.at("homeSet").as_bool();
    return false; // Default if not available
}

int TelemetryReceiver::get_remaining_flight_time() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("remainingFlightTime")) 
    {
        const auto& remaining_flight_time = telemetry.at("remainingFlightTime");
        if (remaining_flight_time.is_int64())
            return static_cast<int>(remaining_flight_time.as_int64());
        if (remaining_flight_time.is_uint64())
            return static_cast<int>(remaining_flight_time.as_uint64());
    }
    return 0; // Default if not available
}

double TelemetryReceiver::get_time_needed_to_go_home() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("timeNeededToGoHome") && telemetry.at("timeNeededToGoHome").is_double())
        return boost::json::value_to<double>(telemetry.at("timeNeededToGoHome"));
    return 0.0; // Default if not available
}

double TelemetryReceiver::get_time_needed_to_land() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("timeNeededToLand") && telemetry.at("timeNeededToLand").is_double())
        return boost::json::value_to<double>(telemetry.at("timeNeededToLand"));
    return 0.0; // Default if not available
}

double TelemetryReceiver::get_total_time() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("totalTime") && telemetry.at("totalTime").is_double())
        return boost::json::value_to<double>(telemetry.at("totalTime"));
    return 0.0; // Default if not available
}

double TelemetryReceiver::get_max_radius_can_fly_and_go_home() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("maxRadiusCanFlyAndGoHome") && telemetry.at("maxRadiusCanFlyAndGoHome").is_double())
        return boost::json::value_to<double>(telemetry.at("maxRadiusCanFlyAndGoHome"));
    return 0.0; // Default if not available
}

int TelemetryReceiver::get_remaining_charge() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("remainingCharge")) 
    {
        const auto& remaining_charge = telemetry.at("remainingCharge");
        if (remaining_charge.is_int64())
            return static_cast<int>(remaining_charge.as_int64());
        if (remaining_charge.is_uint64())
            return static_cast<int>(remaining_charge.as_uint64());
    }
    return 0; // Default if not available
}

double TelemetryReceiver::get_battery_needed_to_land() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("batteryNeededToLand") && telemetry.at("batteryNeededToLand").is_double())
        return boost::json::value_to<double>(telemetry.at("batteryNeededToLand"));
    return 0.0; // Default if not available
}

double TelemetryReceiver::get_battery_needed_to_go_home() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("batteryNeededToGoHome") && telemetry.at("batteryNeededToGoHome").is_double())
        return boost::json::value_to<double>(telemetry.at("batteryNeededToGoHome"));
    return 0.0; // Default if not available
}

double TelemetryReceiver::get_serious_low_battery_threshold() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("seriousLowBatteryThreshold") && telemetry.at("seriousLowBatteryThreshold").is_double())
        return boost::json::value_to<double>(telemetry.at("seriousLowBatteryThreshold"));
    return 0.0; // Default if not available
}

double TelemetryReceiver::get_low_battery_threshold() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("lowBatteryThreshold") && telemetry.at("lowBatteryThreshold").is_double())
        return boost::json::value_to<double>(telemetry.at("lowBatteryThreshold"));
    return 0.0; // Default if not available
}

std::string TelemetryReceiver::get_drone_status() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("droneStatus") && telemetry.at("droneStatus").is_string())
        return boost::json::value_to<std::string>(telemetry.at("droneStatus"));
    return ""; 
}

std::string TelemetryReceiver::get_flight_mode() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("flightMode") && telemetry.at("flightMode").is_string())
        return boost::json::value_to<std::string>(telemetry.at("flightMode"));
    return ""; 
}

bool TelemetryReceiver::is_manual_override_active() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("isManualOverrideActive") && telemetry.at("isManualOverrideActive").is_bool())
        return telemetry.at("isManualOverrideActive").as_bool();
    return false; // Default if not available
}

bool TelemetryReceiver::is_virtual_stick_enabled() 
{
    std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
    const auto& telemetry = this->latest_telemetry_;
    if (telemetry.contains("isVirtualStickActive") && telemetry.at("isVirtualStickActive").is_bool())
        return telemetry.at("isVirtualStickActive").as_bool();
    return false; // Default if not available
}


/* Protected Class Methods ---------------------------------------------------*/

void TelemetryReceiver::process_raw_line(const std::string& line) 
{
    try 
    {
        // Try to convert the line to JSON
        boost::json::value jv = boost::json::parse(line);
        
        std::lock_guard<std::mutex> lock(this->telemetry_mtx_);
        this->latest_telemetry_ = std::move(jv.as_object());
    } 
    catch (const std::exception& e) 
    {
        // Ignore malformed JSON lines, but log the error for debugging
    }
}