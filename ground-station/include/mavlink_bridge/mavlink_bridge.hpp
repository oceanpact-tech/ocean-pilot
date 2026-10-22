/**
 ******************************************************************************
 * @file    mavlink_bridge.hpp
 * @brief   MAVLink-to-DJI protocol bridge and Ground Station communication node.
 *
 * This file defines the MavlinkBridge class, responsible for acting as a 
 * bidirectional translator between proprietary DJI drone interfaces and the 
 * standard MAVLink protocol. It exposes MAVSDK server plugins (Telemetry, 
 * Action, Params) to Ground Control Stations (like QGC) and orchestrates
 * specialized sub-modules for state management, telemetry, actions, and control.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <span>
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

// Project modules
#include "dji_interface/dji_interface.hpp"
#include "mavlink_bridge/params/px4_params.hpp"
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/telemetry_publisher.hpp"
#include "mavlink_bridge/flight_action_executor.hpp"
#include "mavlink_bridge/manual_control_handler.hpp"


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
 * The MavlinkBridge encapsulates the MAVSDK servers and orchestrates the 
 * composed domain modules (FlightStateManager, TelemetryPublisher, 
 * FlightActionExecutor, and ManualControlHandler). It manages the connection 
 * lifecycle, schedules the 10 Hz telemetry timer, and routes incoming/outgoing 
 * MAVLink callbacks.
 */
class MavlinkBridge 
{
public:

    /** 
     * @brief Constructor for MavlinkBridge.
     * @param ioc                   Reference to the Boost.Asio I/O context.
     * @param wildbridge_socket     Socket configuration for the WildBridge Android server.
     * @param ground_station_socket Socket configuration for the Ground Control Station.
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
    DJIInterface dji_;                                ///< DJI interface for telemetry and control.


    // --------------------------
    // MAVLINK CONFIG & SERVERS
    // --------------------------

    mavsdk::Mavsdk              mavsdk_;                ///< Core MAVSDK instance managing communication and plugins.
    mavsdk::TelemetryServer     telemetry_server_;      ///< MAVSDK telemetry server.
    mavsdk::ActionServer        action_server_;         ///< MAVSDK action server.
    mavsdk::MavlinkDirectServer mavlink_direct_server_; ///< MAVSDK MAVLink direct server.
    mavsdk::ParamServer         param_server_;          ///< MAVSDK param server.

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

    /// @brief Creates and prepares the allowable flight modes configuration for ActionServer.
    inline static mavsdk::ActionServer::AllowableFlightModes create_allowable_flight_modes_config() 
    {
        mavsdk::ActionServer::AllowableFlightModes allowed_modes{};
        allowed_modes.can_auto_mode         = true;
        allowed_modes.can_guided_mode       = true;
        allowed_modes.can_stabilize_mode    = true;
        allowed_modes.can_auto_land_mode    = true;
        allowed_modes.can_auto_takeoff_mode = true;
        allowed_modes.can_auto_rtl_mode     = true;
        allowed_modes.can_auto_loiter_mode  = true;
        return allowed_modes;
    };

    /// @brief Provides a span of integer parameters to the MAVSDK ParamServer.
    inline void load_params(std::span<const std::pair<const char*, int>> int_params) 
    {
        for (const auto& [name, value] : int_params) {
            this->param_server_.provide_param_int(name, value);
        }
    }

    /// @brief Provides a span of float parameters to the MAVSDK ParamServer.
    inline void load_params(std::span<const std::pair<const char*, float>> float_params) 
    {
        for (const auto& [name, value] : float_params) {
            this->param_server_.provide_param_float(name, value);
        }
    }


    // --------------------------
    // COMPOSED MODULES
    // --------------------------

    FlightStateManager   state_manager_;          ///< Thread-safe state machine for flight modes and maneuvers.
    TelemetryPublisher   telemetry_publisher_;    ///< Telemetry and heartbeat publisher module.
    FlightActionExecutor flight_action_executor_; ///< Asynchronous executor for takeoff, land, and arming.
    ManualControlHandler manual_control_handler_; ///< Handler for DJI Virtual Stick and MANUAL_CONTROL inputs.


    // --------------------------
    // SUBSCRIPTION HANDLES
    // --------------------------

    mavsdk::Mavsdk::InterceptJsonHandle          outgoing_msg_handle_{}; ///< Handle for outgoing message filter.
    mavsdk::Mavsdk::InterceptJsonHandle          incoming_msg_handle_{}; ///< Handle for incoming message router.
    mavsdk::ActionServer::ArmDisarmHandle        arm_disarm_handle_{};   ///< Handle for arm/disarm command callback.
    mavsdk::ActionServer::TakeoffHandle          takeoff_handle_{};      ///< Handle for takeoff command callback.
    mavsdk::ActionServer::FlightModeChangeHandle flight_mode_handle_{};  ///< Handle for flight mode change callback.


    // --------------------------
    // SCHEDULING TIMER
    // --------------------------

    boost::asio::steady_timer telemetry_timer_; ///< Timer for scheduling telemetry sending (declared last to be destroyed first).

    /// @brief Wait for telemetry stream from DJI to become stable.
    void wait_for_telemetry(); 
    
    /// @brief Send telemetry data via MAVLink at a fixed frequency.
    void schedule_telemetry();


    // --------------------------
    // SUBSCRIBERS
    // --------------------------

    /// @brief Intercepts outgoing MAVLink messages to drop unwanted auto-ACKs.
    bool on_outgoing_message(mavsdk::Mavsdk::MavlinkMessage msg);

    /// @brief Intercepts incoming MAVLink messages such as MANUAL_CONTROL.
    bool on_incoming_message(mavsdk::Mavsdk::MavlinkMessage msg);

    /// @brief Handles incoming flight mode change requests from the Ground Station.
    void on_flight_mode_change(mavsdk::ActionServer::FlightMode mode);

    /// @brief Filters outgoing COMMAND_ACK messages to block default MAVSDK takeoff ACKs.
    bool on_command_ack_outgoing_message(mavsdk::Mavsdk::MavlinkMessage msg);
};