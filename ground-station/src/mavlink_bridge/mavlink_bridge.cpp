/**
 ******************************************************************************
 * @file    mavlink_bridge.cpp
 * @brief   Implementation of the MAVLink bridge and Ground Station communication.
 *
 * This file contains the implementation of the MavlinkBridge class. It 
 * orchestrates the MAVSDK server plugins and delegates state management, 
 * telemetry publishing, autonomous maneuvers, and manual control to its 
 * composed domain modules.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/mavlink_bridge.hpp"

#include <iostream>
#include <chrono>

#include "mavlink_bridge/custom_types/mav_cmd.hpp"
#include "mavlink_bridge/utils/mavlink_json_utils.hpp"


/* Aliases -------------------------------------------------------------------*/

using MavlinkMessage = mavsdk::Mavsdk::MavlinkMessage;
using Result         = mavsdk::ActionServer::Result;
using FlightMode     = mavsdk::ActionServer::FlightMode;


/* Public Class Methods ------------------------------------------------------*/

MavlinkBridge::MavlinkBridge(
    net::io_context& ioc, 
    const WildBridgeSocketConfig wildbridge_socket,
    const GroundStationSocketConfig ground_station_socket
) 
    : ground_station_socket_(std::move(ground_station_socket)),
      dji_(ioc, std::move(wildbridge_socket)),
      mavsdk_(this->create_mavsdk_config()),
      telemetry_server_(this->mavsdk_.server_component()),
      action_server_(this->mavsdk_.server_component()),
      mavlink_direct_server_(this->mavsdk_.server_component()),
      param_server_(this->mavsdk_.server_component()),
      state_manager_(),
      telemetry_publisher_(
          this->dji_,
          this->state_manager_,
          this->telemetry_server_,
          this->action_server_,
          this->mavlink_direct_server_
      ),
      flight_action_executor_(
          this->dji_,
          this->state_manager_,
          this->mavlink_direct_server_
      ),
      manual_control_handler_(
          this->dji_,
          this->state_manager_
      ),
      telemetry_timer_(ioc)
{ 
    this->dji_.start_telemetry_stream();
}

MavlinkBridge::~MavlinkBridge() 
{
    // Stop scheduling timer
    this->stop();

    // Unsubscribe all MAVSDK callbacks before composed modules are destroyed
    this->mavsdk_.unsubscribe_outgoing_messages_json(this->outgoing_msg_handle_);
    this->mavsdk_.unsubscribe_incoming_messages_json(this->incoming_msg_handle_);
    this->action_server_.unsubscribe_arm_disarm(this->arm_disarm_handle_);
    this->action_server_.unsubscribe_takeoff(this->takeoff_handle_);
    this->action_server_.unsubscribe_flight_mode_change(this->flight_mode_handle_);

    // Stop telemetry stream from DJI
    this->dji_.stop_telemetry_stream();
}

bool MavlinkBridge::start() 
{
    // Action server setup
    this->action_server_.set_allowable_flight_modes(this->create_allowable_flight_modes_config());
    this->action_server_.set_allow_takeoff(false); // todo: testar efeito do bool
    this->action_server_.set_armable(true, true);    
    this->action_server_.set_disarmable(true, true);  
    this->action_server_.set_flight_mode(FlightMode::Hold);

    // Load parameters
    this->load_params(oceanpilot::Params::int_params);
    this->load_params(oceanpilot::Params::float_params);

    // Connect to ground station
    mavsdk::ConnectionResult conn = this->mavsdk_.add_any_connection(
        "udpout://" + this->ground_station_socket_.ip + ":" + this->ground_station_socket_.port);
    
    if (conn != mavsdk::ConnectionResult::Success) {
        std::cerr << "[MavlinkBridge] Erro ao abrir conexão: " << conn << '\n'; ///! REMOVE IN RELEASE
        return false;
    }
    std::cout << "[MavlinkBridge] Conexão aberta. Heartbeats sendo enviados para o QGC...\n"; ///! REMOVE IN RELEASE
    
    // Start telemetry stream via mavlink
    this->schedule_telemetry();

    // --------------------------
    // SUBSCRIBERS
    // --------------------------

    // Outgoing messages
    this->outgoing_msg_handle_ = this->mavsdk_.subscribe_outgoing_messages_json(
        [this](MavlinkMessage msg) -> bool {
            return this->on_outgoing_message(msg);
        }
    );

    // Incoming messages
    this->incoming_msg_handle_ = this->mavsdk_.subscribe_incoming_messages_json(
        [this](MavlinkMessage msg) -> bool {
            return this->on_incoming_message(msg);
        }
    );

    // Arm/Disarm action
    this->arm_disarm_handle_ = this->action_server_.subscribe_arm_disarm(
        [this](Result result, mavsdk::ActionServer::ArmDisarm command) {
            this->flight_action_executor_.handle_arm_disarm(command);
        }
    );

    // Takeoff action
    this->takeoff_handle_ = this->action_server_.subscribe_takeoff(
        [this](Result result, bool /*clear_all*/) {
            this->flight_action_executor_.execute_takeoff();
        }
    );

    // Mode change action (also handles QGC Land requests)
    this->flight_mode_handle_ = this->action_server_.subscribe_flight_mode_change(
        [this](Result result, FlightMode mode) {
            this->on_flight_mode_change(mode);
        }
    );

    return true;
}

void MavlinkBridge::stop()
{
    this->telemetry_timer_.cancel();
}


/* Private Class Methods -----------------------------------------------------*/

// --------------------------
// TELEMETRY SCHEDULING
// --------------------------

void MavlinkBridge::wait_for_telemetry()
{
    // Check if the first telemetry data was received
    if (this->dji_.is_telemetry_ready()) 
    {
        std::cout << "[MavlinkBridge] Telemetry ready. Starting to send data...\n";
        this->schedule_telemetry(); 
        return;
    }

    this->telemetry_timer_.expires_after(std::chrono::milliseconds(500));
    this->telemetry_timer_.async_wait([this](const boost::system::error_code& ec) {
        if (!ec) 
        {
            std::cout << "[MavlinkBridge] Waiting for telemetry stream to connect...\n";
            this->wait_for_telemetry(); 
        }
    });
}

void MavlinkBridge::schedule_telemetry() 
{
    this->telemetry_timer_.expires_after(std::chrono::milliseconds(1000 / TELEMETRY_FREQUENCY_HZ));
    this->telemetry_timer_.async_wait([this](const boost::system::error_code& ec) {
        if (!ec) 
        {
            this->manual_control_handler_.maintain_virtual_stick();
            this->telemetry_publisher_.publish_cycle();

            this->schedule_telemetry(); 
        }
    });
}

// --------------------------
// SUBSCRIBERS
// --------------------------

bool MavlinkBridge::on_outgoing_message(MavlinkMessage msg)
{
    if (msg.message_name == "COMMAND_ACK")
        return this->on_command_ack_outgoing_message(msg);

    return true; // Allow any other message
}

bool MavlinkBridge::on_incoming_message(MavlinkMessage msg)
{
    if (msg.message_name == "MANUAL_CONTROL")
    {
        std::cout << "ENTREI AQUI!!\n";
        return this->manual_control_handler_.handle_manual_control_message(msg);
    }

    if (msg.message_name == "COMMAND_LONG")
    {
        auto command = extract_json_value(msg.fields_json, "command");
    
        std::string do_reposition_code = std::to_string(MavCmd::MAV_CMD_DO_REPOSITION);

        if (command == std::nullopt)
            return false;
        
        if (command == do_reposition_code)
        {
            auto param7_str = extract_json_value(msg.fields_json, "param7");
            if (param7_str)
            {
                float target_amsl = std::stof(param7_str.value());
                this->flight_action_executor_.execute_change_altitude(target_amsl);
            }
                
            return false;
        }

        
    }

    return true; // Pass any other message.
}

void MavlinkBridge::on_flight_mode_change(FlightMode mode)
{
    switch (mode)
    {
        case FlightMode::Hold: 
            std::cout << "[MavlinkBridge] MUDAR Modo para: HOLD\n";
            this->state_manager_.request_flight_mode(FlightMode::Hold);
            break;

        case FlightMode::Posctl:
            std::cout << "[MavlinkBridge] MUDAR Modo para: POSITION\n";
            this->state_manager_.request_flight_mode(FlightMode::Posctl);
            break;

        case FlightMode::Land:
            std::cout << "[MavlinkBridge] MUDAR Modo para: LAND\n";
            this->flight_action_executor_.execute_land();
            break;

        case FlightMode::ReturnToLaunch:
            std::cout << "[MavlinkBridge] MUDAR Modo para: RETURN TO LAUNCH\n";
            this->flight_action_executor_.execute_return_to_launch();
            break;

        case FlightMode::Takeoff:
            std::cout << "[MavlinkBridge] MUDAR Modo para: TAKEOFF\n";
            break;

        default:
            std::cout << "[MavlinkBridge] Modo não suportado solicitado. Revertendo no próximo ciclo.\n";
            this->telemetry_publisher_.invalidate_published_mode();
            break;
    }
}

bool MavlinkBridge::on_command_ack_outgoing_message(MavlinkMessage msg)
{
    auto command = extract_json_value(msg.fields_json, "command");
    
    std::string arm_disarm_code = std::to_string(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
    std::string takeoff_code    = std::to_string(MavCmd::MAV_CMD_NAV_TAKEOFF);
    
    if (command == std::nullopt)
        return false;
    
    if (command == takeoff_code || command == arm_disarm_code)
    {
        auto result_param2 = extract_json_value(msg.fields_json, "result_param2");
        if (result_param2 == "999")
            return true;
        return false;
    }

    return true;
}