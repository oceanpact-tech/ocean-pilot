/**
 ******************************************************************************
 * @file    mavlink_bridge.hpp
 * @brief   MAVLink-to-DJI protocol bridge and Ground Station communication node.
 *
 * This file defines the MavlinkBridge class, responsible for acting as a 
 * bidirectional translator between proprietary DJI drone interfaces and the 
 * standard MAVLink protocol. It exposes MAVSDK server plugins (Telemetry, 
 * Action, Params) to Ground Control Stations (like QGC).
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <atomic>
#include <thread>
#include <memory>
#include <span>
#include <optional>
#include <string>

// Asio
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>

// MAVSDK
#include <mavsdk/mavsdk.hpp>
#include <mavsdk/compatibility_mode.hpp>
#include <mavsdk/autopilot.hpp>

#include <mavsdk/plugins/telemetry_server/telemetry_server.hpp>
#include <mavsdk/plugins/action_server/action_server.hpp>
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>
#include <mavsdk/plugins/param_server/param_server.hpp>

#include "dji_interface/dji_interface.hpp"
#include "mavlink_bridge/params/px4_params.hpp"
#include "mavlink_bridge/custom_types/mav_result.hpp" 

/* Definitions  --------------------------------------------------------------*/

#define TELEMETRY_FREQUENCY_HZ 10 // Frequency of telemetry sending in Hz


/* Aliases  ------------------------------------------------------------------*/

namespace net = boost::asio;


/* Structs  ------------------------------------------------------------------*/

struct GroundStationSocketConfig{
    std::string ip;
    std::string port;
};


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Core bridging node between the DJI hardware interface and the MAVLink network.
 * 
 * The MavlinkBridge encapsulates the MAVSDK servers and asynchronously manages
 * the data flow between the drone and the Ground Control Station (GCS). 
 * It continuously polls raw telemetry from the DJI interface, formats it into 
 * MAVLink standards, and handles incoming GCS commands (arming, modes, parameters).
 */
class MavlinkBridge 
{
public:

    /** 
     * @brief Constructor for MavlinkBridge.
     * @param ground_station_socket
     * @param wildbridge_socket
     */
    MavlinkBridge(
        net::io_context& ioc, 
        const WildBridgeSocketConfig wildbridge_socket,
        const GroundStationSocketConfig ground_station_socket
    );

    /// @brief Destructor for MavlinkBridge.
    ~MavlinkBridge();

    /// @brief Start connection to ground station.
    bool start();

    /// @brief Stop connection to ground station.
    void stop();

private:

    GroundStationSocketConfig ground_station_socket_; ///< Ground station socket.
    DJIInterface dji_; ///< DJI interface for telemetry and control.

    // --------------------------
    // MAVLINK CONFIG
    // --------------------------

    mavsdk::Mavsdk mavsdk_; ///< Core MAVSDK instance managing MAVLink communication and server plugins.
 
    inline static uint32_t system_id_    = 0;
    inline static uint8_t  component_id_ = 0;

    /// @brief Creates and prepares the MAVSDK configuration.
    inline static mavsdk::Mavsdk::Configuration create_mavsdk_config()
    {   
        mavsdk::Mavsdk::Configuration config(mavsdk::ComponentType::Autopilot); // Configure as an Autopilot component.
        
        // Setters
        config.set_mav_type(mavsdk::MavType::Quadrotor);
        config.set_autopilot(mavsdk::Autopilot::Px4);
        config.set_compatibility_mode(mavsdk::CompatibilityMode::Px4);

        // Getters
        MavlinkBridge::system_id_    = config.get_system_id();
        MavlinkBridge::component_id_ = config.get_component_id();

        return config;
    };


    // --------------------------
    // TELEMETRY SERVER
    // --------------------------

    mavsdk::TelemetryServer telemetry_server_; ///< MAVSDK telemetry server.

    // Telemetry sending mechanism
    boost::asio::steady_timer telemetry_timer_;     ///< Timer for scheduling telemetry sending.
    std::atomic<bool> is_telemetry_running_{false}; ///< Flag to indicate if telemetry is running.
    std::thread telemetry_thread_;                  ///< Thread for sending telemetry.

    // Raw buffers for telemetry
    Location3D raw_location_buffer_; ///< Raw location data from DJIInterface.
    Speed3D    raw_speed_buffer_;    ///< Raw speed data from DJIInterface.
    double     raw_heading_buffer_;  ///< Raw heading data from DJIInterface.
    Attitude3D raw_attitude_buffer_; ///< Raw attitude data from DJIInterface.


    // Buffers for telemetry
    mavsdk::TelemetryServer::Position            position_buffer_;         ///< Buffer for position telemetry.
    mavsdk::TelemetryServer::VelocityNed         velocity_buffer_;         ///< Buffer for velocity telemetry.                   
    mavsdk::TelemetryServer::Heading             heading_buffer_;          ///< Buffer for heading  telemetry.
    mavsdk::TelemetryServer::EulerAngle          attitude_buffer_;         ///< Buffer for attitude telemetry.
    mavsdk::TelemetryServer::AngularVelocityBody angular_velocity_buffer_; ///< Buffer for angular velocity telemetry.


    /// @brief Wait for telemetry stream from DJI is stable.
    void wait_for_telemetry(); 
    
    /// @brief Send telemetry data via mavlink at a fixed frequeny.
    void schedule_telemetry();

    /// @brief Get raw telemetry from DJI.
    void get_raw_telemetry();

    /// @brief Populate telemetry buffers with data received from DJI.
    void prepare_mavlink_telemetry();

    /// @brief Publish telemetry to mavlink.
    void publish_telemetry();

    void publish_heartbeat();

    void publish_sys_status();

    void publish_extended_sys_state();

    void maintain_virtual_stick();

    // --------------------------
    // ACTION SERVER
    // --------------------------

    mavsdk::ActionServer action_server_; ///< MAVSDK action server.

    std::atomic<mavsdk::ActionServer::FlightMode> previous_flight_mode_{
        mavsdk::ActionServer::FlightMode::Hold
    };
    
    std::atomic<bool> should_virtual_stick_be_enabled_{false};
    std::atomic<bool> is_virtual_stick_enabling_{false};
    std::atomic<bool> is_virtual_stick_enabled_{false};
    std::atomic<bool> is_on_takeoff{false};
    std::atomic<bool> is_on_land{false};

    /// @brief Creates and prepares the MAVSDK configuration.
    inline static mavsdk::ActionServer::AllowableFlightModes create_allawable_fligh_modes_config() 
    {
        mavsdk::ActionServer::AllowableFlightModes allowed_modes{};
        allowed_modes.can_auto_mode         = true;
        allowed_modes.can_guided_mode       = true;
        allowed_modes.can_stabilize_mode    = true;
        allowed_modes.can_auto_land_mode    = true;
        allowed_modes.can_auto_takeoff_mode = true;
        allowed_modes.can_auto_rtl_mode     = false;
        allowed_modes.can_auto_loiter_mode  = false;
        return allowed_modes;
    };


    // --------------------------
    // MAVLINK DIRECT SERVER
    // --------------------------

    mavsdk::MavlinkDirectServer mavlink_direct_server_; ///< MAVSDK MAVlink direct server.


    // --------------------------
    // PARAM SERVER
    // --------------------------
    
    mavsdk::ParamServer param_server_; ///< MAVSDK param server.

    inline void load_params(std::span<const std::pair<const char*, int>> int_params) 
    {
        for (const auto& [name, value] : int_params) {
            this->param_server_.provide_param_int(name, value);
        }
    }

    inline void load_params(std::span<const std::pair<const char*, float>> float_params) 
    {
        for (const auto& [name, value] : float_params) {
            this->param_server_.provide_param_float(name, value);
        }
    }


    // --------------------------
    // SUBSCRIBERS
    // --------------------------

    /// @brief Intercepts outgoing MAVLink messages to drop unwanted auto-ACKs.
    bool on_outgoing_message(mavsdk::Mavsdk::MavlinkMessage msg);

    bool on_incoming_message(mavsdk::Mavsdk::MavlinkMessage msg);

    void on_flight_mode_change(mavsdk::ActionServer::FlightMode mode);

    /// @brief Handles incoming Arm/Disarm commands from the Ground Station.
    void on_arm_disarm_command(mavsdk::ActionServer::Result result, mavsdk::ActionServer::ArmDisarm command);

    void on_takeoff(mavsdk::ActionServer::Result result, bool clear_all);

    void on_land(mavsdk::ActionServer::Result result, bool clear_all);

    bool on_manual_control(mavsdk::Mavsdk::MavlinkMessage msg);

    // --------------------------
    // HELPERS
    // --------------------------

    /// @brief Create a formatted mavlink message to be send as COMMAND_ACK.
    mavsdk::MavlinkDirectServer::MavlinkMessage create_ack_mavlink_msg(uint16_t command, uint8_t resul);

    /// @brief Get the value for a specific key from json string.
    std::optional<std::string> extract_json_value(const std::string& json_str, const std::string& key);
};