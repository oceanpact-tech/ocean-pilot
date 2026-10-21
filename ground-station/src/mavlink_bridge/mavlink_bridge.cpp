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

#include "mavlink_bridge/custom_types/mav_cmd.hpp"
#include "mavlink_bridge/custom_types/mav_result.hpp"
#include "mavlink_bridge/custom_types/mav_land_state.hpp"


/* Aliases -------------------------------------------------------------------*/

using MavlinkMessage = mavsdk::Mavsdk::MavlinkMessage;
using DirectMavlinkMessage = mavsdk::MavlinkDirectServer::MavlinkMessage;
using Result = mavsdk::ActionServer::Result;
using FlightMode = mavsdk::ActionServer::FlightMode;


/* Prototypes ----------------------------------------------------------------*/

DirectMavlinkMessage create_ack_mavlink_msg(uint16_t command, uint8_t result);
std::optional<std::string> extract_json_value(const std::string& json_str, const std::string& key);


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
    this->action_server_.set_allow_takeoff(false);
    this->action_server_.set_armable(true, true);    
    this->action_server_.set_disarmable(true, true);  
    this->action_server_.set_flight_mode(FlightMode::Hold); // Set current flight mode

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
    this->mavsdk_.subscribe_outgoing_messages_json(
        [this](MavlinkMessage msg) -> bool {
            return this->on_outgoing_message(msg);
        }
    );

    // Incoming messages
    this->mavsdk_.subscribe_incoming_messages_json(
        [this](MavlinkMessage msg) -> bool {
            return this->on_incoming_message(msg);
        }
    );

    // Arm/Disarm action
    this->action_server_.subscribe_arm_disarm(
        [this](Result result, mavsdk::ActionServer::ArmDisarm command) {
            return this->on_arm_disarm_command(result, command);
        }
    );

    // Takeoff
    this->action_server_.subscribe_takeoff(
        [this](Result result, bool clear_all) {
            return this->on_takeoff(result, clear_all);
        }
    );

    // Land action required by ground station engage subscribe_flight_mode_change callback.

    // Mode change
    this->action_server_.subscribe_flight_mode_change(
        [this](Result result, FlightMode mode) {
            return this->on_flight_mode_change(mode);
        }
    );

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
            this->publish_heartbeat();
            this->publish_sys_status();
            this->publish_extended_sys_state();
            this->maintain_virtual_stick();

            this->get_raw_telemetry();
            this->prepare_mavlink_telemetry();
            this->publish_telemetry();

            this->schedule_telemetry(); 
        }
    });
}

void MavlinkBridge::get_raw_telemetry() 
{
    this->raw_home_location_buffer = this->dji_.get_home_location();
    this->raw_home_altitude_buffer = this->dji_.get_home_altitude();
    this->raw_location_buffer_     = this->dji_.get_location();
    this->raw_speed_buffer_        = this->dji_.get_speed();
    this->raw_heading_buffer_      = this->dji_.get_heading();
    this->raw_attitude_buffer_     = this->dji_.get_attitude(); 
}

void MavlinkBridge::prepare_mavlink_telemetry()
{    
    // Home location 
    this->home_location_buffer_.latitude_deg        = this->raw_home_location_buffer.latitude;
    this->home_location_buffer_.longitude_deg       = this->raw_home_location_buffer.longitude;
    this->home_location_buffer_.absolute_altitude_m = this->raw_home_altitude_buffer;
    this->home_location_buffer_.relative_altitude_m = 0.0;

    // Position
    this->position_buffer_.latitude_deg        = this->raw_location_buffer_.latitude;
    this->position_buffer_.longitude_deg       = this->raw_location_buffer_.longitude;
    this->position_buffer_.absolute_altitude_m = this->raw_location_buffer_.altitude;
    this->position_buffer_.relative_altitude_m = this->raw_location_buffer_.altitude + this->raw_home_altitude_buffer;

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

void MavlinkBridge::publish_telemetry()
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

void MavlinkBridge::publish_heartbeat()
{
    // Update arming state
    this->action_server_.set_armed_state(this->dji_.is_armed());

    FlightMode target_mode = FlightMode::Hold;
    if (this->is_on_takeoff_)
        target_mode = FlightMode::Takeoff;
    else if (this->is_on_land_)
        target_mode = FlightMode::Land;
    else if (this->should_virtual_stick_be_enabled_ || 
             (this->is_virtual_stick_enabled_ && this->dji_.is_armed()))
        target_mode = FlightMode::Posctl;

    static FlightMode current_mode = FlightMode::Unknown;

    // Update flight mode
    if (current_mode != target_mode)
    {
        current_mode = target_mode;
        this->action_server_.set_flight_mode(target_mode);
    }
}
    
void MavlinkBridge::publish_sys_status()
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

void MavlinkBridge::publish_extended_sys_state()
{
    uint8_t landed_state = 
        this->dji_.is_armed() ? 
        MavLandState::MAV_LANDED_STATE_IN_AIR :
        MavLandState::MAV_LANDED_STATE_ON_GROUND;

    mavsdk::MavlinkDirectServer::MavlinkMessage msg;
    msg.message_name = "EXTENDED_SYS_STATE";
    msg.fields_json = R"({
        "vtol_state": 0,
        "landed_state": )" + std::to_string(landed_state) + R"(
    })";

    this->mavlink_direct_server_.send_message(msg);
}

void MavlinkBridge::maintain_virtual_stick()
{
    // Update inner state
    bool is_virtual_stick_enabled = this->dji_.is_virtual_stick_enabled();
    bool is_armed = this->dji_.is_armed();

    this->is_virtual_stick_enabled_ = is_virtual_stick_enabled;
    this->is_armed_ = is_armed;

    // If pilot takes the control back, take POSITION mode down and go back to HOLD
    if (this->dji_.is_manual_override_active())
    {
        this->should_virtual_stick_be_enabled_ = false;
        this->previous_flight_mode_ = FlightMode::Hold;
        return;
    }

    // Disable virtual stick
    if (!this->should_virtual_stick_be_enabled_ || 
        !is_armed                               ||
        this->is_on_takeoff_                    ||
        this->is_on_land_)
    {
        if (is_virtual_stick_enabled)
            this->dji_.disable_virtual_stick();
        return;
    }

    if (!is_virtual_stick_enabled)
    {
        static auto last_attempt = std::chrono::steady_clock::now() - std::chrono::seconds(5);
        auto now = std::chrono::steady_clock::now();

        // Retry enable virtual stick every 1.5s
        if (!this->is_virtual_stick_enabling_ && 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_attempt).count() >= 1500)
        {
            this->is_virtual_stick_enabling_ = true;
            last_attempt = now;

            this->dji_.enable_virtual_stick([this](bool success) {
                this->is_virtual_stick_enabling_ = false;
                if (!success) {
                    std::cerr << "[MavlinkBridge] Falha HTTP ao enviar enableVirtualStick\n";
                }
            });
        }     
    }
}

// --------------------------
// SUBSCRIBERS
// --------------------------

bool MavlinkBridge::on_outgoing_message(MavlinkMessage msg)
{
    if (msg.message_name == "COMMAND_ACK")
        this->on_command_ack_outgoing_message(msg);

    return true; // Allow any other message
}

bool MavlinkBridge::on_incoming_message(MavlinkMessage msg)
{
    
    if (msg.message_name == "MANUAL_CONTROL")
    {
        return this->on_manual_control(msg);
    }

    std::cout << msg.fields_json << "\n";

    return true;
}

void MavlinkBridge::on_flight_mode_change(FlightMode mode)
{
    if (mode == this->previous_flight_mode_)
        return;

    switch (mode)
    {
        case FlightMode::Hold: 
            std::cout << "[MavlinkBridge] MUDAR Modo para: HOLD\n";
            this->should_virtual_stick_be_enabled_ = false;
            this->previous_flight_mode_ = mode;
            break;

        case FlightMode::Posctl:
            std::cout << "[MavlinkBridge] MUDAR Modo para: POSITION\n";
            this->previous_flight_mode_ = mode;
            this->should_virtual_stick_be_enabled_ = true;
            break;

        case FlightMode::Land:
            std::cout << "[MavlinkBridge] MUDAR Modo para: LAND\n";
            this->on_land(Result::Success, true);
            break;

        case FlightMode::Acro:
            std::cout << "[MavlinkBridge] MUDAR Modo para: ACRO\n";
            break; 

        case FlightMode::Altctl:
            std::cout << "[MavlinkBridge] MUDAR Modo para: ALTCTL\n";
            break; 
        
        case FlightMode::FollowMe:
            std::cout << "[MavlinkBridge] MUDAR Modo para: FOLLOW ME\n";
            break; 

        case FlightMode::Manual:
            std::cout << "[MavlinkBridge] MUDAR Modo para: MANUAL\n";
            break; 
        
        case FlightMode::Mission:
            std::cout << "[MavlinkBridge] MUDAR Modo para: MISSION\n";
            break; 

        case FlightMode::Offboard:
            std::cout << "[MavlinkBridge] MUDAR Modo para: OFFBOARD\n";
            break; 

        case FlightMode::Ready:
            std::cout << "[MavlinkBridge] MUDAR Modo para: READY\n";
            break; 

        case FlightMode::ReturnToLaunch:
            std::cout << "[MavlinkBridge] MUDAR Modo para: RETURN TO LAUNCH\n";
            break; 

        case FlightMode::Stabilized:
            std::cout << "[MavlinkBridge] MUDAR Modo para: STABILIZED\n";
            break; 
        
        case FlightMode::Takeoff:
            std::cout << "[MavlinkBridge] MUDAR Modo para: TAKEOFF\n";
            break; 

        default:
            break;
    }
}

void MavlinkBridge::on_arm_disarm_command(Result result, mavsdk::ActionServer::ArmDisarm command) 
{
    // Command = DISARM
    if (command.arm)
    {
        if (!this->dji_.is_ready_to_takeoff())
        {
            this->mavlink_direct_server_.send_message(
                create_ack_mavlink_msg(
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 
                    MavResult::DENIED
                )
            );
            return;
        };

        this->mavlink_direct_server_.send_message(
            create_ack_mavlink_msg(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 
                MavResult::ACCEPTED
            )
        );
        return;
    }

    // Command = DISARM
    if (!command.arm)
    {
        return;
    }
}

void MavlinkBridge::on_takeoff(Result result, bool clear_all)
{
    if (this->is_on_takeoff_.exchange(true))
        return; // Avoid release more than one thread

    const FlightMode mode_before_takeoff = 
        this->previous_flight_mode_.exchange(FlightMode::Land);
    this->should_virtual_stick_be_enabled_ = false; // Security only, since takeoff is required when previous mode is HOLD (still on the ground).

    auto abort_takeoff = [this, mode_before_takeoff]() {
        this->previous_flight_mode_ = mode_before_takeoff;
        this->should_virtual_stick_be_enabled_ = (mode_before_takeoff == FlightMode::Posctl); // Security only.
        this->is_on_takeoff_ = false;
        this->action_server_.set_flight_mode(mode_before_takeoff);
    };

    std::thread([this, mode_before_takeoff, abort_takeoff]() 
    {
        if (!this->dji_.is_ready_to_takeoff())
        {
            this->mavlink_direct_server_.send_message(
                create_ack_mavlink_msg(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::DENIED
                )
            );
            abort_takeoff();
            return;
        };
        
        auto http_success = std::make_shared<std::atomic<bool>>(false);
        auto http_done    = std::make_shared<std::atomic<bool>>(false);

        this->dji_.request_takeoff([http_success, http_done](bool success){
            *http_success = success;
            *http_done    = true;
        });
        
        bool takeoff_started = false;
        
        // Wait takeoff start for 10 seconds.
        for (int i = 0; i < 40; i++)
        {
            if (*http_done && !(*http_success))
                break; 
            
            if (this->dji_.get_drone_status() == "TAKING_OFF")
            {
                takeoff_started = true;
                this->should_virtual_stick_be_enabled_ = false;
                break;
            }

            this->mavlink_direct_server_.send_message(
                create_ack_mavlink_msg(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::IN_PROGRESS
                )
            );

            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

        // If takeoff has not started, send FAILED.
        if (!takeoff_started)
        {
            this->mavlink_direct_server_.send_message(
                create_ack_mavlink_msg(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::FAILED
                )
            );
            abort_takeoff();
            return;
        }

        // Send IN_PROGRESS untill takeoff is done.
        while (!(this->dji_.get_drone_status() == "IDLE"))
        {
            this->mavlink_direct_server_.send_message(
                create_ack_mavlink_msg(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::IN_PROGRESS
                )
            );
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Send ACCEPTED when task is over.
        this->mavlink_direct_server_.send_message(
            create_ack_mavlink_msg(
                MavCmd::MAV_CMD_NAV_TAKEOFF, 
                MavResult::ACCEPTED
            )
        );

        this->is_on_takeoff_ = false;

        // Drone MUST engage to Hold mode when auto takeoff is done.
        // When calling set_flight_mode(Hold), next previous_flight_mode will be Hold and should_virtual_stick_be_enabled_ will be false.
        this->action_server_.set_flight_mode(FlightMode::Hold);
    }).detach();
}

void MavlinkBridge::on_land(Result result, bool clear_all)
{
    if (this->is_on_land_.exchange(true))
        return; // Avoid release more than one thread

    const FlightMode mode_before_land = this->previous_flight_mode_;
    this->should_virtual_stick_be_enabled_ = false;

    auto abort_land = [this, mode_before_land]() {
        this->previous_flight_mode_ = mode_before_land;
        this->should_virtual_stick_be_enabled_ = (mode_before_land == FlightMode::Posctl);
        this->is_on_land_ = false;
        this->action_server_.set_flight_mode(mode_before_land);
    };

    std::thread([this, abort_land]() 
    {
        if (!this->dji_.is_armed())
        {
            abort_land();
            return;
        };

        auto http_success = std::make_shared<std::atomic<bool>>(false);
        auto http_done    = std::make_shared<std::atomic<bool>>(false);

        this->dji_.request_land([http_success, http_done](bool success){
            *http_success = success;
            *http_done    = true;
        });

        bool landing_started = false;

        // Wait land start for 3 seconds.
        for (int i = 0; i < 12; i++)
        {
            if (*http_done && !(*http_success))
                break; 

            if (this->dji_.get_drone_status() == "LANDING")
            {
                landing_started  = true;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

        // If land has not started, send FAILED.
        if (!landing_started)
        {
            abort_land();
            return;
        }

        // Send IN_PROGRESS untill land is done.
        while (this->dji_.get_drone_status() != "IDLE" &&
               this->dji_.is_armed())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

        this->is_on_land_ = false;

        // Drone MUST engage to Hold mode when auto land is done.
        // When calling set_flight_mode(Hold), next previous_flight_mode will be Hold and should_virtual_stick_be_enabled_ will be false.
        this->action_server_.set_flight_mode(FlightMode::Hold);
    }).detach();
}

bool MavlinkBridge::on_manual_control(MavlinkMessage msg)
{
    if (!this->is_virtual_stick_enabled_)
        return true;

    auto lx_str = extract_json_value(msg.fields_json, "r"); // Yaw   (-1000 to 1000)
    auto ly_str = extract_json_value(msg.fields_json, "z"); // Throttle (0 to 1000)
    auto rx_str = extract_json_value(msg.fields_json, "y"); // Roll  (-1000 to 1000)
    auto ry_str = extract_json_value(msg.fields_json, "x"); // Pitch (-1000 to 1000)

    if (rx_str && ry_str && lx_str && ly_str) 
    {
        // Convert to float
        float lx = std::stof(lx_str.value());
        float ly = std::stof(ly_str.value());
        float rx = std::stof(rx_str.value());
        float ry = std::stof(ry_str.value());

        lx = (lx / 1000.f) / 2.f;
        ly = ((ly - 500.f )/ 500.f) / 2.f;
        rx = (rx / 1000.f) / 2.f;
        ry = (ry / 1000.f) / 2.f;

        std::string payload = std::to_string(lx) + "," + 
                                std::to_string(ly) + "," + 
                                std::to_string(rx) + "," +
                                std::to_string(ry);

        static auto last_send = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();

        // Send at 20 Hz
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count() >= 50) 
        {
            this->dji_.send_stick(payload);
            last_send = now;
        }

        return true;
    }
    return false;
}

bool MavlinkBridge::on_command_ack_outgoing_message(MavlinkMessage msg)
{
    std::optional<std::string> command = extract_json_value(msg.fields_json, "command");
    
    std::string arm_disarm_code = std::to_string(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
    std::string takeoff_code    = std::to_string(MavCmd::MAV_CMD_NAV_TAKEOFF);
    std::string land_code       = std::to_string(MavCmd::MAV_CMD_NAV_LAND);
    
    if (command == std::nullopt)
        return false;
    
    if (command == takeoff_code)
    {
        std::optional<std::string> result_param2 = extract_json_value(msg.fields_json, "result_param2");
        if(result_param2 == "999")
            return true;
        return false;
    }
    else
        return true;
}

// --------------------------
// HELPERS
// --------------------------

DirectMavlinkMessage create_ack_mavlink_msg(uint16_t command, uint8_t result)
{
    DirectMavlinkMessage msg;
    msg.message_name = "COMMAND_ACK";
    msg.fields_json  = R"({
        "command":)" + std::to_string(command) + R"(,
        "result":)" + std::to_string(result) + R"(,
        "progress":0,
        "result_param2":999,
        "target_component":0,
        "target_system":255
    })";
    return msg;
}

std::optional<std::string> extract_json_value(const std::string& json_str, const std::string& key) 
{
    std::string search_key = "\"" + key + "\":";

    size_t key_pos = json_str.find(search_key);
    if (key_pos == std::string::npos) 
        return std::nullopt;
    
    size_t value_start = key_pos + search_key.length();
    if (value_start >= json_str.length()) 
        return std::nullopt;

    // Case 1: value is a string
    if (json_str[value_start] == '"') 
    {
        size_t text_start = value_start + 1; 
        size_t text_end = json_str.find('"', text_start);
        if (text_end == std::string::npos) 
            return std::nullopt;
        return json_str.substr(text_start, text_end - text_start);
    } 
    // Case 2: value is a bool or a number
    else 
    {
        size_t value_end = json_str.find_first_of(",}", value_start);
        if (value_end == std::string::npos) 
            return std::nullopt;
        return json_str.substr(value_start, value_end - value_start);
    }
}