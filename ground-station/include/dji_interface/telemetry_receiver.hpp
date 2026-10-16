/**
 ******************************************************************************
 * @file    telemetry_handler.hpp
 * @brief   Parses and provides access to DJI NDJSON telemetry data.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */

#pragma once


/* Includes ------------------------------------------------------------------*/

#include <mutex>
#include <string>

// Boost
#include <boost/json.hpp>

#include "dji_interface/transport/tcp_handler.hpp"
#include "dji_interface/custom_types/custom_types.hpp"


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Specializes TcpHandler to decode WildBridge JSON telemetry.
 */
class TelemetryReceiver : public TcpHandler 
{
public:
    /**
     * @brief Constructor for the TelemetryReceiver class.
     * 
     * @param ioc  Reference to the application's central asynchronous event loop.
     * @param host Target IP address or hostname of the WildBridge server.
     * @param port Target port number (default is "8081").
     */
    TelemetryReceiver(
        net::io_context& ioc, 
        std::string host, 
        std::string port = "8081"
    )
        : TcpHandler(
            ioc, 
            std::move(host), 
            std::move(port)
        ) {}

    /// @brief
    ~TelemetryReceiver() { this->stop_telemetry(); }

    /// @brief Starts the continuous NDJSON telemetry stream in the background.
    inline void start_telemetry() { this->start_comm(); }

    /// @brief Stops the telemetry stream gracefully
    inline void stop_telemetry()  { this->stop_comm(); }

    /// @brief Get latest telemetry
    boost::json::object get_telemetry();

    /// @brief Check if telemetry data has been received and parsed.
    bool is_telemetry_ready();

    /**
     * @brief Get all aircraft states from telemetry.
     * @param verbose If true, prints the telemetry to stdout.
     */
    boost::json::object request_all_states(bool verbose = false);
    
    // Telemetry field accessors ----------------

    /// @brief Get aircraft name.
    std::string get_drone_name();

    /// @brief Get aircraft velocity (x, y, z).
    Speed3D get_speed();

    /// @brief Get compass heading in degrees.
    double get_heading();
    
    /// @brief Get aircraft attitude (pitch, roll, yaw) in degrees.
    Attitude3D get_attitude();
    
    /// @brief Get aircraft 3D location (latitude, longitude, altitude). GPS position.
    Location3D get_location();
    
    /// @brief Get gimbal attitude (pitch, roll, yaw) in degrees.
    Attitude3D get_gimbal_attitude();
    
    /// @brief Get gimbal joint attitude (pitch, roll, yaw) in degrees.
    Attitude3D get_gimbal_joint_attitude();

    /// @brief Get camera zoom focal length.
    double get_zoom_focal_length();
    
    /// @brief Get camera hybrid focal length.
    double get_hybrid_focal_length();
    
    /// @brief Get camera optical focal length.
    double get_optical_focal_length();
    
    /// @brief Get camera zoom ratio.
    double get_zoom_ratio();

    /// @brief Get battery level percentage.
    int get_battery_level();
    
    /// @brief Get GPS satellite count.
    int get_satellite_count();

    /// @brief Whether the drone is currently armed (derived on the aircraft side).
    bool is_armed();

    /// @brief Whether the drone is ready to take off / arm (derived on the aircraft side).
    bool is_ready_to_takeoff();

    /// @brief Reason the drone cannot take off: DJIDeviceStatus name, 'NONE', or 'UNKNOWN'.
    std::string get_takeoff_block_reason();

    /// @brief Get home location (latitude, longitude).
    Location2D get_home_location();
    
    /// @brief Get distance to home in meters.
    double get_distance_to_home();

    /// @brief Get the last LRF-locked target position (latitude, longitude, altitude).
    Location3D get_lrf_target();

    /** 
     * @brief Id of the waypoint the streamed 'waypointReached' currently refers to.
     *
     * Mirrors DroneController._waypointSeq, incremented by the app for every
     *requestSendGoToWaypointNoseForward. Returns -1 if telemetry hasn't reported it yet.
     */
    int get_waypoint_seq();

    /** 
     * @brief Check if a commanded waypoint has been reached.
     * @param seq
     */
    bool is_waypoint_reached(int seq);
    
    /// @brief Check if an intermediary waypoint has been reached.
    bool is_intermediary_waypoint_reached();
    
    /// @brief Id of the gotoYaw command the streamed 'yawReached' refers to (-1 if unknown).
    int get_yaw_seq();

    /** 
     * @brief Check if a commanded yaw has been reached.
     *
     * 'yawReached' latches until the next command, so pass the seq returned by
     * requestSendGotoYaw to require the streamed status belongs to that command.
     * seq=None keeps the legacy (race-prone) raw-flag read.
     * 
     * @param seq
     */
    bool is_yaw_reached(int seq);

    /// @brief Id of the gotoAltitude command the streamed 'altitudeReached' refers to (-1 if unknown).
    int get_altitude_seq();

    /** 
     * @brief Check if a commanded altitude has been reached.
     *
     * Pass the seq returned by requestSendGotoAltitude to avoid the stale-latch race.
     * seq=None keeps the legacy (race-prone) raw-flag read.
     * @param seq
     */
    bool is_altitude_reached(int seq);
    
    /// @brief Check if the camera is currently recording.
    bool is_camera_recording();
    
    /// @brief Check if the home location has been set.
    bool is_home_set();
    
    /// @brief Get remaining flight time in minutes.
    int get_remaining_flight_time();
    
    /// @brief Get time needed to return home in seconds.
    double get_time_needed_to_go_home();
    
    /// @brief Get time needed to land in seconds.
    double get_time_needed_to_land();
    
    /// @brief Get total time needed (go home + land) in seconds.
    double get_total_time();
    
    /// @brief Get maximum radius the drone can fly and still return home.
    double get_max_radius_can_fly_and_go_home();
    
    /// @brief Get remaining battery charge percentage.
    int get_remaining_charge();
    
    /// @brief Get battery percentage needed to land.
    double get_battery_needed_to_land();
    
    /// @brief Get battery percentage needed to return home.
    double get_battery_needed_to_go_home();
    
    /// @brief Get serious low battery warning threshold percentage.
    double get_serious_low_battery_threshold();
    
    /// @brief Get low battery warning threshold percentage.
    double get_low_battery_threshold();
    
    /// @brief Get the current drone status (e.g., 'IDLE', 'ARMING', 'TAKING_OFF', etc.).
    std::string get_drone_status();

    /// @brief Get the current flight mode (e.g., 'MANUAL', 'GPS', 'GO_HOME', etc.).
    std::string get_flight_mode();

    /** @brief Check if manual override is active (pilot took RC control).
     *
     * When True, autonomous HTTP commands are being rejected by the app.
     * The pilot must deactivate manual override before autonomous commands work again.
     */
    bool is_manual_override_active();

    /// @brief Check if virtual stick is enabled.
    bool is_virtual_stick_enabled();

protected:
    /**
     * @brief Overrides the base class hook to parse the NDJSON.
     */
    void process_raw_line(const std::string& line) override;

private:
    std::mutex telemetry_mtx_; /// Telemetry mutex
    boost::json::object latest_telemetry_; /// Lastest telemetry data received from the drone

    /**
     * @brief Safely extracts a double from a boost::json::value, handling integer types.
     * @param val The JSON value to parse.
     * @return The extracted number as double, or 0.0 if the type is invalid.
     */
    inline double jv_to_double(const boost::json::value& val)
    {
        if (val.is_double()) return val.as_double();
        if (val.is_int64())  return static_cast<double>(val.as_int64());
        if (val.is_uint64()) return static_cast<double>(val.as_uint64());
        return 0.0; // Default if it is not a number
    };
};