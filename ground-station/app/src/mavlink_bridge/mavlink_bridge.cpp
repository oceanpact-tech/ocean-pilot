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

#include <cmath>
#include <iostream>
#include <chrono>

// Mavlink types
#include "mavlink_bridge/custom_types/mav_cmd.hpp"
#include "mavlink_bridge/custom_types/mav_result.hpp"
#include "mavlink_bridge/custom_types/px4_flight_modes.hpp"

// Helpers
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
      // DJI API
      dji_(ioc, std::move(wildbridge_socket)),

      // MAVSDK API
      autopilot_component_(this->create_autopilot_component_config()),
      telemetry_server_(this->autopilot_component_.server_component()),
      action_server_(this->autopilot_component_.server_component()),
      mavlink_direct_server_(this->autopilot_component_.server_component()),
      param_server_(this->autopilot_component_.server_component()),
      mission_server_(this->autopilot_component_.server_component()),

      // ACTUATORS
      camera_server_(this->autopilot_component_.server_component_by_type(mavsdk::ComponentType::Camera)),
      gimbal_direct_server_(this->autopilot_component_.server_component_by_type(mavsdk::ComponentType::Camera)),
    //   gimbal_component_(this->create_gimbal_component_config()),

      
      // Composed modules: same order as the declaration in the header (maneuver_runner_ last).
      state_manager_(
        // Do not use DJI Interface
        // Do not use MAVSDK servers
        this->flight_action_executor_, this->flight_primitives_,
        this->mission_handler_, this->mission_executor_, 
        this->ground_station_logger_, this->manual_control_handler_, this->telemetry_publisher_,
        this->maneuver_runner_
      ),
      flight_action_executor_(
        this->dji_, 
        this->mavlink_direct_server_,
        this->state_manager_, this->flight_primitives_,
        this->mission_handler_, this->mission_executor_, 
        this->ground_station_logger_, this->manual_control_handler_, this->telemetry_publisher_,
        this->maneuver_runner_
      ),
      flight_primitives_(
        this->dji_, 
        // Do not use MAVSDK servers
        this->state_manager_, this->flight_action_executor_,
        this->mission_handler_, this->mission_executor_, 
        this->ground_station_logger_, this->manual_control_handler_, this->telemetry_publisher_,
        this->maneuver_runner_
      ),
      mission_handler_(
        // Do not use DJI Interface
        this->mission_server_,
        this->state_manager_, this->flight_action_executor_, this->flight_primitives_,
        this->mission_executor_, 
        this->ground_station_logger_, this->manual_control_handler_, this->telemetry_publisher_,
        this->maneuver_runner_
      ),
      mission_executor_(
        this->dji_,
        // Do not use MAVSDK servers
        this->state_manager_, this->flight_action_executor_, this->flight_primitives_,
        this->mission_handler_, 
        this->ground_station_logger_, this->manual_control_handler_, this->telemetry_publisher_,
        this->maneuver_runner_
      ),
      ground_station_logger_(
        // Do not use DJI Interface
        this->mavlink_direct_server_,
        this->state_manager_, this->flight_action_executor_, this->flight_primitives_,
        this->mission_handler_, this->mission_executor_,
        this->manual_control_handler_, this->telemetry_publisher_,
        this->maneuver_runner_
      ),
      manual_control_handler_(
        this->dji_,
        // Do not use MAVSDK servers
        this->state_manager_, this->flight_action_executor_, this->flight_primitives_,
        this->mission_handler_, this->mission_executor_,
        this->ground_station_logger_, this->telemetry_publisher_,
        this->maneuver_runner_
      ),
      telemetry_publisher_(
        this->dji_,
        this->telemetry_server_, this->action_server_, this->mavlink_direct_server_, this->gimbal_direct_server_,
        this->state_manager_, this->flight_action_executor_, this->flight_primitives_,
        this->mission_handler_, this->mission_executor_,
        this->ground_station_logger_, this->manual_control_handler_,
        this->maneuver_runner_
      ),
      maneuver_runner_(
        this->state_manager_, this->flight_action_executor_, this->flight_primitives_,
        this->mission_handler_, this->mission_executor_,
        this->ground_station_logger_, this->manual_control_handler_, this->telemetry_publisher_
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
    this->autopilot_component_.unsubscribe_outgoing_messages_json(this->outgoing_msg_handle_);
    this->autopilot_component_.unsubscribe_incoming_messages_json(this->incoming_msg_handle_);
    this->action_server_.unsubscribe_arm_disarm(this->arm_disarm_handle_);
    this->action_server_.unsubscribe_takeoff(this->takeoff_handle_);
    this->action_server_.unsubscribe_flight_mode_change(this->flight_mode_handle_);

    // Join the maneuver thread while every module is still alive
    this->maneuver_runner_.cancel();

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

    // Connect autopilot 
    mavsdk::ConnectionResult conn = this->autopilot_component_.add_any_connection(
        "udpout://" + this->ground_station_socket_.ip + ":" + this->ground_station_socket_.port);
    
    if (conn != mavsdk::ConnectionResult::Success) {
        std::cerr << "[MavlinkBridge] Failure to connect autopilot: " << conn << '\n'; ///! REMOVE IN RELEASE
        return false;
    }

    // // Connect gimbal
    // mavsdk::ConnectionResult gimbal_conn = this->gimbal_component_.add_any_connection(
    // "udpout://" + this->ground_station_socket_.ip + ":" + this->ground_station_socket_.port);

    // if (gimbal_conn != mavsdk::ConnectionResult::Success) {
    //     std::cerr << "[MavlinkBridge] Failure to connect gimbal: " << gimbal_conn << '\n';
    //     return false;
    // }

    std::cout << "[MavlinkBridge] Successfully connected. Heartbeats being sent to the QGC...\n"; ///! REMOVE IN RELEASE

    // No seu método start(), antes de iniciar o telemetry_timer:
    mavsdk::CameraServer::Information cam_info{};
    
    cam_info.vendor_name = "DJI";
    cam_info.model_name  = "Zenmuse M30";
    cam_info.horizontal_resolution_px = 1920;
    cam_info.vertical_resolution_px   = 1080;

    this->camera_server_.set_information(cam_info);

    // Start telemetry stream via mavlink
    this->schedule_telemetry();

    // Start mission handler
    this->mission_handler_.start();

    // --------------------------
    // SUBSCRIBERS
    // --------------------------

    // Outgoing messages
    this->outgoing_msg_handle_ = this->autopilot_component_.subscribe_outgoing_messages_json(
        [this](MavlinkMessage msg) -> bool {
            return this->on_outgoing_message(msg);
        }
    );

    // Incoming messages
    this->incoming_msg_handle_ = this->autopilot_component_.subscribe_incoming_messages_json(
        [this](MavlinkMessage msg) -> bool {
            return this->on_incoming_message(msg);
        }
    );

    // Arm/Disarm action
    this->arm_disarm_handle_ = this->action_server_.subscribe_arm_disarm(
        [this](Result /*result*/, mavsdk::ActionServer::ArmDisarm command) {
            // set_armed_state() in the telemetry cycle re-enters here with force = true: ignore that echo.
            if (command.force)
                return;
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



    this->storage_info_handle_ = this->camera_server_.subscribe_storage_information(
        [this](int32_t /*storage_id*/) {
            mavsdk::CameraServer::StorageInformation s{};
            s.storage_id     = 1;
            s.storage_status = mavsdk::CameraServer::StorageInformation::StorageStatus::NotAvailable;
            // storage_type e os campos de MiB/velocidade ficam em 0 (default) quando não há cartão

            auto res = this->camera_server_.respond_storage_information(
                mavsdk::CameraServer::CameraFeedback::Ok, s);

            if (res != mavsdk::CameraServer::Result::Success)
                std::cerr << "[MavlinkBridge] respond_storage_information failed: " << res << '\n';
        }
    );


    camera_server_.subscribe_take_photo([this](int32_t index) {
        // chamar WildBridge HTTP aqui
        mavsdk::CameraServer::CaptureInfo info{};
        info.index = index;
        info.is_success = true;
        camera_server_.respond_take_photo(mavsdk::CameraServer::CameraFeedback::Ok, info);
    });

    camera_server_.subscribe_start_video([this](int32_t /*stream_id*/) {
        camera_server_.respond_start_video(mavsdk::CameraServer::CameraFeedback::Ok);
    });
    camera_server_.subscribe_stop_video([this](int32_t /*stream_id*/) {
        camera_server_.respond_stop_video(mavsdk::CameraServer::CameraFeedback::Ok);
    });


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

            this->check_gimbal_watchdog(); /// GIMBAL

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

    if (msg.message_name == "HEARTBEAT")
    {
        // auto autopilot_component_str = extract_json_value(msg.fields_json, "autopilot");
        // auto custom_mode_str = extract_json_value(msg.fields_json, "custom_mode");
        
        // if (!autopilot_component_str || !custom_mode_str) return false;
        // if (autopilot_component_str.value() != "12") return false;

        // try {
        //     uint32_t custom_mode = std::stoul(custom_mode_str.value());
            
        //     bool is_my_custom_mode = (
        //         custom_mode == PX4FlightModes::PX4_CUSTOM_MODE_POSCTL           || // Position
        //         custom_mode == PX4FlightModes::PX4_CUSTOM_MODE_HOLD             || // Hold
        //         custom_mode == PX4FlightModes::PX4_CUSTOM_MODE_MISSION          || // Mission
        //         custom_mode == PX4FlightModes::PX4_CUSTOM_MODE_RETURN_TO_LAUNCH || // RTL
        //         custom_mode == PX4FlightModes::PX4_CUSTOM_MODE_LAND                // Land
        //     );

        //     if (!is_my_custom_mode) {
        //         return false;
        //     }
        // } catch (...) {
        //     return false; 
        // }
        
        return true;
    }
    
    return true; // Allow any other message
}

bool MavlinkBridge::on_incoming_message(MavlinkMessage msg)
{
    if (msg.message_name == "MANUAL_CONTROL")
        return this->manual_control_handler_.handle_manual_control_message(msg);

    if (msg.message_name == "COMMAND_LONG")
        return this->on_command_long_incoming_message(msg);

    return true; // Pass any other message.
}

bool MavlinkBridge::on_command_long_incoming_message(const MavlinkMessage& msg)
{
    auto command = extract_json_value(msg.fields_json, "command");

    if (command == std::nullopt)
        return false;

    // Change altitude
    if (command == std::to_string(MavCmd::MAV_CMD_DO_REPOSITION))
    {
        auto param7_str = extract_json_value(msg.fields_json, "param7");
        if (param7_str)
        {
            const float target_amsl = std::stof(param7_str.value());
            if (std::isfinite(target_amsl))
            {
                std::cout << "SOLICITANDO MUDANÇA DE ALTURA\n";
                this->flight_action_executor_.execute_change_altitude(target_amsl);
            }
        }
        return false;
    }

    // Start (or resume) the mission. Items range (param1/param2) is not supported yet.
    if (command == std::to_string(MavCmd::MAV_CMD_MISSION_START))
    {
        const bool accepted = this->mission_executor_.start() || this->mission_executor_.is_active();

        this->mavlink_direct_server_.send_message(
            create_ack_mavlink_msg(
                MavCmd::MAV_CMD_MISSION_START,
                accepted ? MavResult::ACCEPTED : MavResult::DENIED
            )
        );
        return false;
    }

    if (command == "512") // MAV_CMD_REQUEST_MESSAGE
    { 
        auto id = extract_json_value(msg.fields_json, "param1");
        auto tc = extract_json_value(msg.fields_json, "target_component");
        std::cout << "[REQ_MSG] id=" << id.value_or("?")
                  << " target_comp=" << tc.value_or("?") << '\n';
    }

    // QGC solicitando o controle primário do Gimbal
    if (command == "1001") // MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
    {
        this->gimbal_direct_server_.send_message(
            create_ack_mavlink_msg(1001, MavResult::ACCEPTED)
        );
        return false; // Command consumed, do not forward to autopilot.
    }

    // Movimentação do Gimbal (Click & Drag ou Botões)
    if (command == "1000") // MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    {
        auto p1 = extract_json_value(msg.fields_json, "param1"); // Pitch (degrees)
        auto p2 = extract_json_value(msg.fields_json, "param2"); // Yaw (degrees)
        auto p3 = extract_json_value(msg.fields_json, "param3"); // Pitch rate (degrees/s)
        auto p4 = extract_json_value(msg.fields_json, "param4"); // Yaw rate (degrees/s)

        // Converte de std::string para float (trata "NaN" perfeitamente no C++)
        float pitch_angle = p1 != "null" ? std::stof(p1.value()) : NAN;
        float yaw_angle    = p2 != "null" ? std::stof(p2.value()) : NAN;
        // float pitch_rate = p3 != "null" ? std::stof(p3.value()) : NAN;
        // float yaw_rate   = p4 != "null" ? std::stof(p4.value()) : NAN;
        
        // PS.: Use angles as rates.
        float pitch_rate = pitch_angle;
        float yaw_rate   = yaw_angle;

        // ACTION: Center or Split 90 degress.
        if ((p1 == "0.0"   && p2 == "0.0") ||
            (p1 == "-90.0" && p2 == "0.0"))
        {
            this->dji_.set_gimbal_position(0.0, pitch_rate, yaw_rate);
        }
        // ACTION: Click and drag.
        else
        {
            if (std::isnan(pitch_rate)) 
                {
                    pitch_rate = 0.0f;
                }
                else 
            {
                if (pitch_rate > 2.0f)  pitch_rate = 2.0f;
                if (pitch_rate < -2.0f) pitch_rate = -2.0f;
            }

            if (std::isnan(yaw_rate)) 
            {
                yaw_rate = 0.0f;
            }
            else 
            {
                if (yaw_rate > 2.0f)  yaw_rate = 2.0f;
                if (yaw_rate < -2.0f) yaw_rate = -2.0f;
            }
            pitch_rate *= 10.0f; // Ajuste de escala para DJI
            yaw_rate *= 10.0f;   // Ajuste de escala para DJI
            this->dji_.set_gimbal_speed(0.0f, pitch_rate, yaw_rate);
        }
        
        // Update watchdog timer and flag
        this->last_gimbal_cmd_time_ = std::chrono::steady_clock::now();
        this->is_gimbal_moving_ = true;

        // Send ACK to GCS
        this->gimbal_direct_server_.send_message(
            create_ack_mavlink_msg(1000, MavResult::ACCEPTED)
        );

        return false; // Command consumed, do not forward to autopilot.
    }

    return true; // Forward any other command to autopilot.
}

void MavlinkBridge::on_flight_mode_change(FlightMode mode)
{
    switch (mode)
    {
        case FlightMode::Hold: 
            std::cout << "[MavlinkBridge] MUDAR Modo para: HOLD\n";
            this->mission_executor_.pause(); // Pause = hold position, keeping the progress.
            this->state_manager_.request_flight_mode(FlightMode::Hold);
            break;

        case FlightMode::Posctl:
            std::cout << "[MavlinkBridge] MUDAR Modo para: POSITION\n";
            this->mission_executor_.pause();
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
            // Echo of the heartbeat; the takeoff itself arrives as MAV_CMD_NAV_TAKEOFF.
            break;

        case FlightMode::Mission:
            std::cout << "[MavlinkBridge] MUDAR Modo para: MISSION\n";
            // Refused request (not an echo): revert the mode shown to the GCS.
            if (!this->mission_executor_.start() && !this->mission_executor_.is_active())
                this->telemetry_publisher_.invalidate_published_mode();
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

void MavlinkBridge::check_gimbal_watchdog()
{
    if (!this->is_gimbal_moving_)
        return; 

    auto now = std::chrono::steady_clock::now();
    
    if ((now - this->last_gimbal_cmd_time_) > GIMBAL_WATCHDOG_TIMEOUT) 
    {
        // std::cout << "[GIMBAL WATCHDOG] Timeout! Stopping gimbal.\n"; // DEBUG
        this->dji_.set_gimbal_stop(); 
        this->is_gimbal_moving_ = false; 
    }
}