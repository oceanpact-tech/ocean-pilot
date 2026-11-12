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

// Composed modules
#include "mavlink_bridge/flight_state_manager.hpp"

// Mavlink types
#include "mavlink_bridge/custom_types/mav_land_state.hpp"
#include "mavlink_bridge/custom_types/mav_mode.hpp"
#include "mavlink_bridge/custom_types/px4_flight_modes.hpp"


#define TELEMETRY_FREQUENCY_HZ 10 // Frequency of telemetry sending in Hz

/* Aliases -------------------------------------------------------------------*/

using FlightMode = mavsdk::ActionServer::FlightMode;


/* Public Class Methods ------------------------------------------------------*/

TelemetryPublisher::TelemetryPublisher(
    // DJI API
    DJIInterface& dji,
    // MAVSDK API
    mavsdk::TelemetryServer& telemetry_server,
    mavsdk::ActionServer& action_server,
    mavsdk::MavlinkDirectServer& mavlink_direct_server,
    mavsdk::MavlinkDirectServer& gimbal_direct_server,
    // Composed modules
    FlightStateManager&   state_manager,
    FlightActionExecutor& flight_action_executor,
    FlightPrimitives&     flight_primitives,
    MissionHandler&       mission_handler,
    MissionExecutor&      mission_executor,
    GroundStationLogger&  ground_station_logger,
    ManualControlHandler& manual_control_handler,
    ManeuverRunner&       maneuver_runner
)
    : // DJI API
      dji_(dji),
      // MAVSDK API
      telemetry_server_(telemetry_server),
      action_server_(action_server),
      mavlink_direct_server_(mavlink_direct_server),
      gimbal_direct_server_(gimbal_direct_server),
      // Composed modules
      state_manager_(state_manager),
      flight_action_executor_(flight_action_executor),
      flight_primitives_(flight_primitives),
      mission_handler_(mission_handler),
      mission_executor_(mission_executor),
      ground_station_logger_(ground_station_logger),
      manual_control_handler_(manual_control_handler),
      maneuver_runner_(maneuver_runner)
{ }

void TelemetryPublisher::publish_cycle()
{
    this->publish_heartbeat();
    this->publish_sys_status();
    this->publish_extended_sys_state();

    this->get_raw_telemetry();
    this->prepare_mavlink_telemetry();
    this->publish_telemetry();

    this->publish_gimbal(); // TEST
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
    const bool is_armed = this->dji_.is_armed();
    const bool is_vstick_on = this->dji_.is_virtual_stick_enabled();
    const FlightMode target_mode = this->state_manager_.get_target_flight_mode(is_armed, is_vstick_on);

    // Allowed flight modes
    uint32_t px4_custom_mode = 0; 
    switch (target_mode)
    {
        case FlightMode::Posctl:         px4_custom_mode = PX4FlightModes::PX4_CUSTOM_MODE_POSCTL;           break; // Position
        case FlightMode::Hold:           px4_custom_mode = PX4FlightModes::PX4_CUSTOM_MODE_HOLD;             break; // Hold
        case FlightMode::Mission:        px4_custom_mode = PX4FlightModes::PX4_CUSTOM_MODE_MISSION;          break; // Mission
        case FlightMode::ReturnToLaunch: px4_custom_mode = PX4FlightModes::PX4_CUSTOM_MODE_RETURN_TO_LAUNCH; break; // RTL
        case FlightMode::Land:           px4_custom_mode = PX4FlightModes::PX4_CUSTOM_MODE_LAND;             break; // Land
        default:                         px4_custom_mode = PX4FlightModes::PX4_CUSTOM_MODE_HOLD;             break; // Hold
    }

    uint8_t base_mode = MavMode::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; 
    if (is_armed) {
        base_mode |= MavMode::MAV_MODE_FLAG_SAFETY_ARMED;
    }

    constexpr int STANDBY = 3;
    constexpr int ACTIVE  = 4;

    uint8_t system_status = is_armed ? ACTIVE : STANDBY;

    mavsdk::MavlinkDirectServer::MavlinkMessage msg;
    msg.message_name = "HEARTBEAT";
    msg.fields_json = R"({
        "type": 2,
        "autopilot": 12,
        "base_mode": )" + std::to_string(base_mode) + R"(,
        "custom_mode": )" + std::to_string(px4_custom_mode) + R"(,
        "system_status": )" + std::to_string(system_status) + R"(,
        "mavlink_version": 3
    })";

    // Update inner MAVSDK state machine
    this->action_server_.set_armed_state(is_armed);
    const bool force_republish = this->force_mode_republish_.exchange(false);
    if (force_republish || this->last_published_mode_ != target_mode)
    {
        this->last_published_mode_ = target_mode;
        this->action_server_.set_flight_mode(target_mode);
    }

    // Request to hearbeat
    this->mavlink_direct_server_.send_message(msg);
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

void TelemetryPublisher::publish_gimbal()
{
    using Msg = mavsdk::MavlinkDirectServer::MavlinkMessage;
    static const auto t0 = std::chrono::steady_clock::now();
    static uint32_t cycle = 0;

    const auto ms = std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count());

    // ~1 Hz: informações do gimbal manager
    if (cycle++ % TELEMETRY_FREQUENCY_HZ == 0) {
        Msg info;
        info.message_name = "GIMBAL_MANAGER_INFORMATION";
        info.fields_json = R"({
            "time_boot_ms": )" + ms + R"(,
            "cap_flags": 2016,
            "gimbal_device_id": 1,
            "roll_min": 0.0, "roll_max": 0.0,
            "pitch_min": -2.094, "pitch_max": 0.349,
            "yaw_min": -3.1416, "yaw_max": 3.1416
        })";
        this->gimbal_direct_server_.send_message(info);
    }

    // 10 Hz: status do manager
    Msg status;
    status.message_name = "GIMBAL_MANAGER_STATUS";
    status.fields_json = R"({
        "time_boot_ms": )" + ms + R"(,
        "flags": 0,
        "gimbal_device_id": 1,
        "primary_control_sysid": 0,
        "primary_control_compid": 0,
        "secondary_control_sysid": 0,
        "secondary_control_compid": 0
    })";
    this->gimbal_direct_server_.send_message(status);

    // 10 Hz: atitude (quaternion identidade por enquanto)
    Msg att;
    att.message_name = "GIMBAL_DEVICE_ATTITUDE_STATUS";
    att.fields_json = R"({
        "target_system": 0,
        "target_component": 0,
        "time_boot_ms": )" + ms + R"(,
        "flags": 0,
        "q": [1.0, 0.0, 0.0, 0.0],
        "angular_velocity_x": 0.0,
        "angular_velocity_y": 0.0,
        "angular_velocity_z": 0.0,
        "failure_flags": 0,
        "gimbal_device_id": 1
    })";
    this->gimbal_direct_server_.send_message(att);
}