/**
 ******************************************************************************
 * @file    mavlink_bridge.cpp
 * @brief   Implementation of the MAVLink bridge and Ground Station communication.
 *
 * This file contains the implementation of the MavlinkBridge class. It handles
 * the asynchronous execution of MAVSDK servers, processes incoming telemetry 
 * polling from the DJI interface, and manages the continuous dispatch of 
 * MAVLink messages to the connected Ground Control Station (GCS).
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/mavlink_bridge.hpp"

#include <iostream>
#include <functional>
#include <chrono>
#include <iomanip>


/* Public Class Methods ------------------------------------------------------*/

MavlinkBridge::MavlinkBridge(
    net::io_context& ioc, 
    const WildBridgeSocketConfig wildbridge_socket,
    const GroundStationSocketConfig ground_station_socket
) 
    : ground_station_socket_(std::move(ground_station_socket)),
      dji_(ioc, std::move(wildbridge_socket)),
      telemetry_timer_(ioc),
      mavsdk_(this->create_mavsdk_config()),
      telemetry_server_(this->mavsdk_.server_component()),
      action_server_(this->mavsdk_.server_component()),
      mavlink_direct_server_(this->mavsdk_.server_component()),
      param_server_(this->mavsdk_.server_component())
{ 
    this->dji_.start_telemetry_stream();
}

MavlinkBridge::~MavlinkBridge() 
{
    this->dji_.stop_telemetry_stream(); // Stop telemetry stream from DJI
}

bool MavlinkBridge::start() 
{
    // Action server setup
    this->action_server_.set_allowable_flight_modes(this->create_allawable_fligh_modes_config());
    this->action_server_.set_armable(false, false);     // Set armable = false and force_arma = false. This avoid heartbeat be updated by ActionServer.
    this->action_server_.set_disarmable(false, false);  // Set disarmable = false and force_disarm = false.
    this->action_server_.set_flight_mode(mavsdk::ActionServer::FlightMode::Hold); // Set current flight mode

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

    return true;
}

void MavlinkBridge::stop()
{
    this->telemetry_timer_.cancel(); // Stop telemetry stream via mavlink
}


/* Private Class Methods -----------------------------------------------------*/

// --------------------------
// TELEMETRY
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
            auto now = std::chrono::system_clock::now();
            std::time_t current_time = std::chrono::system_clock::to_time_t(now);
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
            this->get_raw_telemetry();
            this->prepare_mavlink_telemetry();
            this->publish_telemetry();
            this->schedule_telemetry(); 
        }
    });
}

void MavlinkBridge::get_raw_telemetry() 
{
    this->raw_location_buffer_ = this->dji_.get_location();
    this->raw_speed_buffer_    = this->dji_.get_speed();
    this->raw_heading_buffer_  = this->dji_.get_heading();
    this->raw_attitude_buffer_ = this->dji_.get_attitude(); 
}

void MavlinkBridge::prepare_mavlink_telemetry()
{    
    // Position
    this->position_buffer_.latitude_deg        = this->raw_location_buffer_.latitude;
    this->position_buffer_.longitude_deg       = this->raw_location_buffer_.longitude;
    this->position_buffer_.absolute_altitude_m = this->raw_location_buffer_.altitude;
    this->position_buffer_.relative_altitude_m = this->raw_location_buffer_.altitude;

    // Speed
    this->velocity_buffer_.north_m_s = this->raw_speed_buffer_.x;
    this->velocity_buffer_.east_m_s  = this->raw_speed_buffer_.y;
    this->velocity_buffer_.down_m_s  = this->raw_speed_buffer_.z;

    // Heading
    this->heading_buffer_.heading_deg = this->raw_heading_buffer_;

    // Angular velocity
    // TODO
}

void MavlinkBridge::publish_telemetry()
{
    this->telemetry_server_.publish_position(
        this->position_buffer_,
        this->velocity_buffer_,
        this->heading_buffer_
    );

    this->telemetry_server_.publish_attitude(
        this->attitude_buffer_, 
        this->angular_velocity_buffer_ // MISSING CONVERTION
    );
}