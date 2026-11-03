/**
 ******************************************************************************
 * @file    flight_action_executor.cpp
 * @brief   Implementation of asynchronous autonomous flight maneuvers.
 *
 * This file implements the FlightActionExecutor class. It handles Arm/Disarm 
 * verification and manages cooperative std::jthread workers for Takeoff and 
 * Land sequences, ensuring safe teardown via std::stop_token and automatic 
 * state restoration via FlightStateManager::ActionScope.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/flight_action_executor.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <utility>

#include "mavlink_bridge/custom_types/mav_cmd.hpp"
#include "mavlink_bridge/custom_types/mav_result.hpp"
#include "mavlink_bridge/utils/mavlink_json_utils.hpp"
#include "mavlink_bridge/utils/http_response_utils.hpp"


/* Public Class Methods ------------------------------------------------------*/

FlightActionExecutor::FlightActionExecutor(
    DJIInterface& dji,
    FlightStateManager& state_manager,
    mavsdk::MavlinkDirectServer& mavlink_direct_server,
    mavsdk::ActionServer& action_server,
    GroundStationLogger& ground_station_logger,
    MissionHandler& mission_handler
)
    : dji_(dji),
      state_manager_(state_manager),
      mavlink_direct_server_(mavlink_direct_server),
      action_server_(action_server),
      gs_logger_(ground_station_logger),
      mission_handler_(mission_handler)
{
}

FlightActionExecutor::~FlightActionExecutor()
{
    if (this->takeoff_worker_.joinable())
    {
        this->takeoff_worker_.request_stop();
        this->takeoff_worker_.join();
    }

    if (this->land_worker_.joinable())
    {
        this->land_worker_.request_stop();
        this->land_worker_.join();
    }

    if (this->rtl_worker_.joinable())
    {
        this->rtl_worker_.request_stop();
        this->rtl_worker_.join();
    }

    if (this->change_altitude_worker_.joinable())
    {
        this->change_altitude_worker_.request_stop();
        this->change_altitude_worker_.join();
    }
}

void FlightActionExecutor::handle_arm_disarm(const mavsdk::ActionServer::ArmDisarm& command)
{
    // Command = ARM
    if (command.arm)
    {
        if (!this->dji_.is_ready_to_takeoff())
        {
            this->send_command_ack(
                MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 
                MavResult::DENIED
            );
            return;
        }

        this->send_command_ack(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 
            MavResult::ACCEPTED
        );
        return;
    }

    // Command = DISARM
    if (!command.arm)
    {
        return;
    }
}

void FlightActionExecutor::execute_takeoff()
{
    auto scope = this->state_manager_.try_begin_maneuver(FlightStateManager::ActiveManeuver::Takeoff);
    if (!scope)
        return; // Another maneuver is already active.

    this->takeoff_worker_ = std::jthread(
        [this, scope = std::move(*scope)](std::stop_token stoken) mutable 
        {
            if (!this->dji_.is_ready_to_takeoff())
            {
                this->send_command_ack(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::DENIED
                );
                return; // ActionScope destructor restores previous mode.
            }

            // Send immediate IN_PROGRESS ACK to prevent QGC 500ms command timeout.
            this->send_command_ack(
                MavCmd::MAV_CMD_NAV_TAKEOFF, 
                MavResult::IN_PROGRESS
            );

            auto http_success = std::make_shared<std::atomic<bool>>(false);
            auto http_done    = std::make_shared<std::atomic<bool>>(false);

            this->dji_.request_takeoff([http_success, http_done](bool success) {
                *http_success = success;
                *http_done    = true;
            });

            bool takeoff_started = false;

            // Wait takeoff start for up to 10 seconds (40 * 250ms).
            for (int i = 0; i < 40 && !stoken.stop_requested(); i++)
            {
                if (*http_done && !(*http_success))
                    break;

                if (this->dji_.get_drone_status() == "TAKING_OFF")
                {
                    takeoff_started = true;
                    break;
                }

                this->send_command_ack(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::IN_PROGRESS
                );

                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            if (stoken.stop_requested())
                return;

            // If takeoff has not started, send FAILED.
            if (!takeoff_started)
            {
                this->send_command_ack(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::FAILED
                );
                return; // ActionScope destructor restores previous mode.
            }

            // Send IN_PROGRESS until takeoff is done.
            while (!stoken.stop_requested() && this->dji_.get_drone_status() != "IDLE")
            {
                this->send_command_ack(
                    MavCmd::MAV_CMD_NAV_TAKEOFF, 
                    MavResult::IN_PROGRESS
                );
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (stoken.stop_requested())
                return;

            // Send ACCEPTED when task is over.
            this->send_command_ack(
                MavCmd::MAV_CMD_NAV_TAKEOFF, 
                MavResult::ACCEPTED
            );

            // Mark maneuver as succeeded so ActionScope transitions to Hold.
            scope.succeed();
        }
    );
}

void FlightActionExecutor::execute_land()
{
    auto scope = this->state_manager_.try_begin_maneuver(FlightStateManager::ActiveManeuver::Land);
    if (!scope)
        return; // Another maneuver is already active.

    this->land_worker_ = std::jthread(
        [this, scope = std::move(*scope)](std::stop_token stoken) mutable 
        {
            if (!this->dji_.is_armed())
                return; // ActionScope destructor restores previous mode.

            auto http_success = std::make_shared<std::atomic<bool>>(false);
            auto http_done    = std::make_shared<std::atomic<bool>>(false);

            this->dji_.request_land([http_success, http_done](bool success) {
                *http_success = success;
                *http_done    = true;
            });

            bool landing_started = false;

            // Wait land start for up to 10 seconds (40 * 250ms).
            for (int i = 0; i < 40 && !stoken.stop_requested(); i++)
            {
                if (*http_done && !(*http_success))
                    break;

                if (this->dji_.get_drone_status() == "LANDING")
                {
                    landing_started = true;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            if (stoken.stop_requested() || !landing_started)
                return; // ActionScope destructor restores previous mode.

            // Wait until landing is done (status is IDLE or motors disarm).
            while (!stoken.stop_requested()                &&
                   this->dji_.get_drone_status() != "IDLE" &&
                   this->dji_.is_armed())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            if (stoken.stop_requested())
                return;

            // Mark maneuver as succeeded so ActionScope transitions to Hold.
            scope.succeed();
        }
    );
}

void FlightActionExecutor::execute_return_to_launch()
{
    auto scope = this->state_manager_.try_begin_maneuver(FlightStateManager::ActiveManeuver::ReturnToLaunch);
    if (!scope)
        return; // Another maneuver is already active.

    this->rtl_worker_ = std::jthread(
        [this, scope = std::move(*scope)](std::stop_token stoken) mutable 
        {
            if (!this->dji_.is_armed())
                return;  // ActionScope destructor restores previous mode.

            auto http_success = std::make_shared<std::atomic<bool>>(false);
            auto http_done    = std::make_shared<std::atomic<bool>>(false);

            this->dji_.request_return_to_home([http_success, http_done](bool success) {
                *http_success = success;
                *http_done    = true;
            });

            bool rtl_started = false;

            // Wait land start for up to 10 seconds (40 * 250ms).
            for (int i = 0; i < 40 && !stoken.stop_requested(); i++)
            {
                if (*http_done && !(*http_success))
                    break;

                if (this->dji_.get_drone_status() == "RETURNING_HOME")
                {
                    rtl_started = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            if (stoken.stop_requested() || !rtl_started)
                return; // ActionScope destructor restores previous mode.

            // Wait until landing is done (status is IDLE or motors disarm).
            while (!stoken.stop_requested() && 
                   this->dji_.get_drone_status() != "IDLE" &&
                   this->dji_.is_armed())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            if (stoken.stop_requested())
                return;

            // Mark maneuver as succeeded so ActionScope transitions to Hold.
            scope.succeed();
        }
    );
}

void FlightActionExecutor::execute_navigate_to_waypoint(const Location3D& waypoint)
{
    // Wait untill previous actions is concluded. Blocking.
    if (this->change_altitude_worker_.joinable())
        this->change_altitude_worker_.join();

    this->is_navigating_to_waypoint_ = true;

    this->navigate_to_waypoint_worker_ = std::jthread(
        [this, waypoint](std::stop_token stoken) 
        {
            if (!this->dji_.is_armed() ||
                this->dji_.is_manual_override_active())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }

            std::string payload = std::to_string(waypoint.latitude/10e6) + "," + std::to_string(waypoint.longitude/10e6) + "," + std::to_string(waypoint.altitude) + "," + std::to_string(this->dji_.get_attitude().yaw) + ",5.0";

            while(true)
            {
                auto http_success = std::make_shared<std::atomic<bool>>(false);
                auto http_done    = std::make_shared<std::atomic<bool>>(false);
                auto response_ptr = std::make_shared<std::string>();

                std::cout << payload << "\n";

                this->dji_.go_to_waypoint_nose_forward(
                    payload,
                    [http_success, http_done, response_ptr](bool success, std::string response) {
                        *http_success = success;
                        *response_ptr = response;
                        *http_done    = true;
                    }
                );

                bool wp_started = false;

                // Wait 10s http conclude.
                for (int i = 0; i < 40 && !stoken.stop_requested(); i++)
                {
                    if (*http_done && !(*http_success))
                        break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }

                for (int i = 0; i < 40 && !stoken.stop_requested(); i++)
                {
                    if (this->dji_.get_drone_status() == "NAVIGATING")
                    {
                        wp_started = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }


                if (stoken.stop_requested())
                    return;
                if (!wp_started)
                    continue;

                std::cout << *response_ptr << "\n";

                // Wait until landing is done (status is IDLE or motors disarm).
                while (!stoken.stop_requested()                 &&
                       this->dji_.get_drone_status() != "IDLE")
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }

                if (stoken.stop_requested())
                    return;

                int seq_id = extract_waypoint_seq(*response_ptr);
                std::cout << "WAYPOINT [" << seq_id << "]: rechead -" << this->dji_.is_waypoint_reached(seq_id) << "\n";

                if (!this->dji_.is_waypoint_reached(seq_id))
                {
                    std::cout << "TENTANDO NOVAMENTE WAYPOINT\n";
                    continue;
                }
                else
                {
                    break;
                }
            }

            this->is_navigating_to_waypoint_ = false;
        }
    );
}

void FlightActionExecutor::execute_change_altitude(const float& target_altitude_amsl)
{
    if (this->is_changing_altitude_)
    {
        this->send_command_ack(
            MavCmd::MAV_CMD_DO_REPOSITION,
            MavResult::TEMPORARILY_REJECTED
        );
        return;
    }

    // Wait untill previous actions is concluded. Blocking.
    if (this->change_altitude_worker_.joinable())
        this->change_altitude_worker_.join();

    this->is_changing_altitude_ = true;

    this->change_altitude_worker_ = std::jthread(
        [this, target_altitude_amsl](std::stop_token /*stoken*/) 
        {
            if (!this->dji_.is_armed() ||
                this->dji_.is_manual_override_active())
            {
                this->send_command_ack(
                    MavCmd::MAV_CMD_DO_REPOSITION, 
                    MavResult::DENIED
                );
                this->is_changing_altitude_ = false;
                return;
            }

            // Send immediate IN_PROGRESS ACK to prevent QGC 500ms command timeout.
            this->send_command_ack(
                MavCmd::MAV_CMD_DO_REPOSITION, 
                MavResult::IN_PROGRESS
            );

            // Calculate target altitude 
            float home_amsl = this->dji_.get_home_altitude();
            float target_altitude_agl = target_altitude_amsl - home_amsl;

            if (target_altitude_agl < 1.0f) 
                target_altitude_agl = 1.0f;

            // Send HTTP requisition
            auto http_success = std::make_shared<std::atomic<bool>>(false);
            auto http_done    = std::make_shared<std::atomic<bool>>(false);

            this->dji_.go_to_altitude_by_position(target_altitude_agl, 
                [http_success, http_done](bool success) {
                    *http_success = success;
                    *http_done    = true;
                });

            bool change_altitude_started = false;

            // Wait change altitude start for up to 10 seconds (40 * 250ms).
            for (int i = 0; i < 40; i++)
            {
                if (*http_done && !(*http_success))
                    break;

                if (this->dji_.get_drone_status() == "NAVIGATING")
                {
                    change_altitude_started = true;
                    break;
                }

                this->send_command_ack(
                    MavCmd::MAV_CMD_DO_REPOSITION, 
                    MavResult::IN_PROGRESS
                );

                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }

            // If change altitude has not started, send FAILED.
            if (!change_altitude_started)
            {
                this->send_command_ack(
                    MavCmd::MAV_CMD_DO_REPOSITION, 
                    MavResult::FAILED
                );
                return;
            }

            // Send IN_PROGRESS until new altitude is reached.
            while (this->dji_.get_drone_status() != "IDLE" &&
                   this->dji_.get_drone_status() != "HOVERING")
            {
                this->send_command_ack(
                    MavCmd::MAV_CMD_DO_REPOSITION, 
                    MavResult::IN_PROGRESS
                );
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            this->send_command_ack(
                MavCmd::MAV_CMD_DO_REPOSITION, 
                MavResult::ACCEPTED
            );
            
            // Allow next request
            this->is_changing_altitude_ = false; 
        }
    );
}

void FlightActionExecutor::execute_mission()
{
    std::vector<mavsdk::MissionRawServer::MissionItem> mission_items =
        this->get_latest_mission_plan().mission_items;

    if (mission_items.empty())
        return;

    if (this->is_on_mission.exchange(true))
        return;

    // Finish running mission before start a new one.
    if (this->mission_worker_.joinable())
        this->mission_worker_.join(); // Wait for running mission thread finish.

    this->mission_worker_ = std::jthread(
        [this, mission_items](std::stop_token /*stoken*/)
        {
            for (const auto& item : mission_items)
            {
                switch (item.command)
                {
                case MavCmd::MAV_CMD_NAV_TAKEOFF:
                    this->execute_takeoff();
                    if (this->takeoff_worker_.joinable())
                        this->takeoff_worker_.join(); // Wait for takeoff finish.
                    this->mission_handler_.set_current_item_complete();
                    break;

                case MavCmd::MAV_CMD_NAV_LAND:
                    this->execute_land();
                    if (this->land_worker_.joinable())
                        this->land_worker_.join(); // Wait for land finish.
                    this->mission_handler_.set_current_item_complete();
                    break;

                case MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    this->execute_return_to_launch();
                    if (this->rtl_worker_.joinable())
                        this->rtl_worker_.join(); // Wait for land finish.
                    this->mission_handler_.set_current_item_complete();
                    break;

                case MavCmd::MAV_CMD_NAV_WAYPOINT:
                {
                    Location3D waypoint;
                    waypoint.latitude  = item.x;
                    waypoint.longitude = item.y;
                    waypoint.altitude  = item.z;
                    
                    this->execute_navigate_to_waypoint(waypoint);
                    
                    if (this->navigate_to_waypoint_worker_.joinable())
                        this->navigate_to_waypoint_worker_.join(); // Wait for land finish.
                    
                        this->mission_handler_.set_current_item_complete();
                    break;
                }

                default:
                    break;
                }
            }
            this->is_on_mission = false;
            this->action_server_.set_flight_mode(mavsdk::ActionServer::FlightMode::Hold);
        }
    );
}

/* Private Class Methods -----------------------------------------------------*/

void FlightActionExecutor::send_command_ack(uint16_t command, uint8_t result)
{
    this->mavlink_direct_server_.send_message(
        create_ack_mavlink_msg(command, result)
    );
}