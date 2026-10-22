/**
 ******************************************************************************
 * @file    telemetry_publisher.cpp
 * @brief   Implementation of MAVLink telemetry, heartbeat, and status publishing.
 *
 * This file contains the implementation of the TelemetryPublisher class. It 
 * polls raw sensor data from the DJIInterface, populates MAVSDK telemetry 
 * structures, and acts as the single writer for Heartbeat flight modes, 
 * SYS_STATUS health bits, and EXTENDED_SYS_STATE messages.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/telemetry_publisher.hpp"

#include <string>

#include "mavlink_bridge/custom_types/mav_land_state.hpp"


/* Aliases -------------------------------------------------------------------*/

using FlightMode = mavsdk::ActionServer::FlightMode;


/* Public Class Methods ------------------------------------------------------*/

TelemetryPublisher::TelemetryPublisher(
    DJIInterface& dji,
    const FlightStateManager& state_manager,
    mavsdk::TelemetryServer& telemetry_server,
    mavsdk::ActionServer& action_server,
    mavsdk::MavlinkDirectServer& mavlink_direct_server
)
    : dji_(dji),
      state_manager_(state_manager),
      telemetry_server_(telemetry_server),
      action_server_(action_server),
      mavlink_direct_server_(mavlink_direct_server)
{
}

void TelemetryPublisher::publish_cycle()
{
    this->publish_heartbeat();
    this->publish_sys_status();
    this->publish_extended_sys_state();

    this->get_raw_telemetry();
    this->prepare_mavlink_telemetry();
    this->publish_telemetry();
}

void TelemetryPublisher::invalidate_published_mode() noexcept
{
    this->force_mode_republish_.store(true);
}


/* Private Class Methods -----------------------------------------------------*/

void TelemetryPublisher::get_raw_telemetry()
{
    this->raw_home_location_buffer_ = this->dji_.get_home_location();
    this->raw_home_altitude_buffer_ = this->dji_.get_home_altitude();
    this->raw_location_buffer_      = this->dji_.get_location();
    this->raw_speed_buffer_         = this->dji_.get_speed();
    this->raw_heading_buffer_       = this->dji_.get_heading();
    this->raw_attitude_buffer_      = this->dji_.get_attitude();
}

void TelemetryPublisher::prepare_mavlink_telemetry()
{
    // Home location
    this->home_location_buffer_.latitude_deg        = this->raw_home_location_buffer_.latitude;
    this->home_location_buffer_.longitude_deg       = this->raw_home_location_buffer_.longitude;
    this->home_location_buffer_.absolute_altitude_m = this->raw_home_altitude_buffer_;
    this->home_location_buffer_.relative_altitude_m = 0.0f;

    // Position
    this->position_buffer_.latitude_deg        = this->raw_location_buffer_.latitude;
    this->position_buffer_.longitude_deg       = this->raw_location_buffer_.longitude;
    this->position_buffer_.relative_altitude_m = this->raw_location_buffer_.altitude;
    this->position_buffer_.absolute_altitude_m = this->raw_location_buffer_.altitude + this->raw_home_altitude_buffer_;

    // Speed
    this->velocity_buffer_.north_m_s = this->raw_speed_buffer_.x;
    this->velocity_buffer_.east_m_s  = this->raw_speed_buffer_.y;
    this->velocity_buffer_.down_m_s  = this->raw_speed_buffer_.z;

    // Heading
    this->heading_buffer_.heading_deg = this->raw_heading_buffer_;

    // Attitude
    this->attitude_buffer_.roll_deg  = this->raw_attitude_buffer_.roll;
    this->attitude_buffer_.pitch_deg = this->raw_attitude_buffer_.pitch;
    this->attitude_buffer_.yaw_deg   = this->raw_attitude_buffer_.yaw;

    // Angular velocity
    // TODO
}

void TelemetryPublisher::publish_telemetry()
{
    this->telemetry_server_.publish_home(
        this->home_location_buffer_
    );

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

void TelemetryPublisher::publish_heartbeat()
{
    const bool is_armed        = this->dji_.is_armed();
    const bool is_vstick_on    = this->dji_.is_virtual_stick_enabled();
    const FlightMode target_mode = this->state_manager_.get_target_flight_mode(is_armed, is_vstick_on);

    // Update arming state
    this->action_server_.set_armed_state(is_armed);

    // Update flight mode if changed or explicitly invalidated
    const bool force_republish = this->force_mode_republish_.exchange(false);
    if (force_republish || this->last_published_mode_ != target_mode)
    {
        this->last_published_mode_ = target_mode;
        this->action_server_.set_flight_mode(target_mode);
    }
}

void TelemetryPublisher::publish_sys_status()
{
    constexpr uint32_t BASE_SENSORS = 64543;
    constexpr uint32_t PREARM_BIT   = 268435456;

    uint32_t sensors_present = BASE_SENSORS | PREARM_BIT;
    uint32_t sensors_enabled = BASE_SENSORS | PREARM_BIT;
    uint32_t sensors_health  = BASE_SENSORS | (this->dji_.is_ready_to_takeoff() ? PREARM_BIT : 0);

    mavsdk::MavlinkDirectServer::MavlinkMessage msg;
    msg.message_name = "SYS_STATUS";
    msg.fields_json = R"({
        "onboard_control_sensors_present": )" + std::to_string(sensors_present) + R"(,
        "onboard_control_sensors_enabled": )" + std::to_string(sensors_enabled) + R"(,
        "onboard_control_sensors_health": )"  + std::to_string(sensors_health) + R"(,
        "load": 0,
        "voltage_battery": 0,
        "current_battery": -1,
        "battery_remaining": -1,
        "drop_rate_comm": 0,
        "errors_comm": 0,
        "errors_count1": 0,
        "errors_count2": 0,
        "errors_count3": 0,
        "errors_count4": 0,
        "onboard_control_sensors_present_extended": 0,
        "onboard_control_sensors_enabled_extended": 0,
        "onboard_control_sensors_health_extended": 0
    })";

    this->mavlink_direct_server_.send_message(msg);
}

void TelemetryPublisher::publish_extended_sys_state()
{
    const uint8_t landed_state = this->state_manager_.get_landed_state(this->dji_.is_armed());

    mavsdk::MavlinkDirectServer::MavlinkMessage msg;
    msg.message_name = "EXTENDED_SYS_STATE";
    msg.fields_json = R"({
        "vtol_state": 0,
        "landed_state": )" + std::to_string(landed_state) + R"(
    })";

    this->mavlink_direct_server_.send_message(msg);
}