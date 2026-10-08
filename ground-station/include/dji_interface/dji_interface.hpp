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

#include "dji_interface/dji_commander.hpp"
#include "dji_interface/telemetry_receiver.hpp"
#include "dji_interface/custom_types/custom_types.hpp"


/* Aliases  ------------------------------------------------------------------*/

namespace net = boost::asio;


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Main interface class for managing communication with the DJI WildBridge.
 * 
 * Interface for DJI drone control via HTTP commands (port 8080) and 
 * TCP telemetry socket (port 8081).
 */
class DJIInterface : public DJICommander
{
public:

    // Constructor
    DJIInterface(
        net::io_context& ioc, 
        const WildBridgeSocketConfig wildbridge_socket
    )
        : DJICommander(
            ioc, wildbridge_socket.ip, wildbridge_socket.command_port),
          telemetry_client_(
            ioc, wildbridge_socket.ip, wildbridge_socket.telemetry_port) 
    {};
    // TODO: Implementar o autodescobrir e iniciar o handle http posteriormente com o ip descoberto

    // Destructor
    ~DJIInterface() {
        this->stop_telemetry_stream();
    };

    /// @brief Starts telemetry stream
    inline void start_telemetry_stream() { this->telemetry_client_.start_comm(); };

    /// @brief Stops telemetry stream
    inline void stop_telemetry_stream() { this->telemetry_client_.stop_comm(); }; 

    /// @brief Check if telemetry data has been received and parsed.
    inline bool is_telemetry_ready() { return this->telemetry_client_.is_telemetry_ready(); }

    // TELEMETRY ACCESSORS -------------------------------------------------------------

    inline boost::json::object request_all_states(bool verbose = false) { return this->telemetry_client_.request_all_states(verbose); }
    inline std::string get_drone_name()                     { return this->telemetry_client_.get_drone_name(); }
    inline Speed3D     get_speed()                          { return this->telemetry_client_.get_speed(); }
    inline double      get_heading()                        { return this->telemetry_client_.get_heading(); }
    inline Attitude3D  get_attitude()                       { return this->telemetry_client_.get_attitude(); }
    inline Location3D  get_location()                       { return this->telemetry_client_.get_location(); }
    inline Attitude3D  get_gimbal_attitude()                { return this->telemetry_client_.get_gimbal_attitude(); }
    inline Attitude3D  get_gimbal_joint_attitude()          { return this->telemetry_client_.get_gimbal_joint_attitude(); }
    inline double      get_zoom_focal_length()              { return this->telemetry_client_.get_zoom_focal_length(); }
    inline double      get_hybrid_focal_length()            { return this->telemetry_client_.get_hybrid_focal_length(); }
    inline double      get_optical_focal_length()           { return this->telemetry_client_.get_optical_focal_length(); }
    inline double      get_zoom_ratio()                     { return this->telemetry_client_.get_zoom_ratio(); }
    inline int         get_battery_level()                  { return this->telemetry_client_.get_battery_level(); }
    inline int         get_satellite_count()                { return this->telemetry_client_.get_satellite_count(); }
    inline bool        is_armed()                           { return this->telemetry_client_.is_armed(); }
    inline bool        is_ready_to_takeoff()                { return this->telemetry_client_.is_ready_to_takeoff(); }
    inline std::string get_takeoff_block_reason()           { return this->telemetry_client_.get_takeoff_block_reason(); }
    inline Location2D  get_home_location()                  { return this->telemetry_client_.get_home_location(); }
    inline double      get_distance_to_home()               { return this->telemetry_client_.get_distance_to_home(); }
    inline Location3D  get_lrf_target()                     { return this->telemetry_client_.get_lrf_target(); }
    inline int         get_waypoint_seq()                   { return this->telemetry_client_.get_waypoint_seq(); }
    inline bool        is_waypoint_reached(int seq)         { return this->telemetry_client_.is_waypoint_reached(seq); }
    inline bool        is_intermediary_waypoint_reached()   { return this->telemetry_client_.is_intermediary_waypoint_reached(); }
    inline int         get_yaw_seq()                        { return this->telemetry_client_.get_yaw_seq(); }
    inline bool        is_yaw_reached(int seq)              { return this->telemetry_client_.is_yaw_reached(seq); }
    inline int         get_altitude_seq()                   { return this->telemetry_client_.get_altitude_seq(); }
    inline bool        is_altitude_reached(int seq)         { return this->telemetry_client_.is_altitude_reached(seq); }
    inline bool        is_camera_recording()                { return this->telemetry_client_.is_camera_recording(); }
    inline bool        is_home_set()                        { return this->telemetry_client_.is_home_set(); }
    inline int         get_remaining_flight_time()          { return this->telemetry_client_.get_remaining_flight_time(); }
    inline double      get_time_needed_to_go_home()         { return this->telemetry_client_.get_time_needed_to_go_home(); }
    inline double      get_time_needed_to_land()            { return this->telemetry_client_.get_time_needed_to_land(); }
    inline double      get_total_time()                     { return this->telemetry_client_.get_total_time(); }
    inline double      get_max_radius_can_fly_and_go_home() { return this->telemetry_client_.get_max_radius_can_fly_and_go_home(); }
    inline int         get_remaining_charge()               { return this->telemetry_client_.get_remaining_charge(); }
    inline double      get_battery_needed_to_land()         { return this->telemetry_client_.get_battery_needed_to_land(); }
    inline double      get_battery_needed_to_go_home()      { return this->telemetry_client_.get_battery_needed_to_go_home(); }
    inline double      get_serious_low_battery_threshold()  { return this->telemetry_client_.get_serious_low_battery_threshold(); }
    inline double      get_low_battery_threshold()          { return this->telemetry_client_.get_low_battery_threshold(); }
    inline std::string get_drone_status()                   { return this->telemetry_client_.get_drone_status(); }
    inline std::string get_flight_mode()                    { return this->telemetry_client_.get_flight_mode(); }
    inline bool        is_manual_override_active()          { return this->telemetry_client_.is_manual_override_active(); }


private:
    TelemetryReceiver telemetry_client_;
};