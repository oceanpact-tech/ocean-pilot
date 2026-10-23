/**
 ******************************************************************************
 * @file    mission_handler.cpp
 * @brief   Implementation of the mission handling and parsing logic.
 *
 * This file implements the MissionHandler class. It extracts waypoint 
 * data and speeds from MAVSDK raw mission plans, converting scaled integers 
 * to decimal degrees, and dispatches them to the Android bridge.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/mission_handler.hpp"

#include <iostream>
#include <format>

#include "mavlink_bridge/custom_types/mav_cmd.hpp"

/* Public Class Methods ------------------------------------------------------*/

MissionHandler::MissionHandler(DJIInterface& dji, mavsdk::MissionRawServer& mission_server)
    : dji_(dji), 
      mission_server_(mission_server)
{
}

MissionHandler::~MissionHandler() 
{
    this->mission_server_.unsubscribe_incoming_mission(this->incoming_mission_handle_);
}

void MissionHandler::start() 
{
    this->incoming_mission_handle_ = this->mission_server_.subscribe_incoming_mission(
        [this](mavsdk::MissionRawServer::Result result, mavsdk::MissionRawServer::MissionPlan plan) 
        {
            if (result != mavsdk::MissionRawServer::Result::Success) 
            {
                std::cerr << "[MissionHandler] Failure to receive mission from QGC: " << result << '\n';
                return;
            }

            if (plan.mission_items.empty()) 
                return;

            float trajectory_speed = 5.0f; // Default speed

            // payload body: "speed;lat1,lon1,alt1;lat2,lon2,alt2..."
            std::string payload = "";
            std::string waypoints_str = "";
            int waypoint_count = 0;

            for (const auto& item : plan.mission_items) 
            {
                // MAV_CMD_DO_CHANGE_SPEED
                if (item.command == MavCmd::MAV_CMD_DO_CHANGE_SPEED && 
                    item.param2 > 0.1f) // Required at least 2 waypoints.
                {
                    trajectory_speed = item.param2;
                }
                // MAV_CMD_NAV_WAYPOINT
                else if (item.command == MavCmd::MAV_CMD_NAV_WAYPOINT) 
                {
                    double lat = item.x / 10000000.0;
                    double lon = item.y / 10000000.0;
                    float  alt = item.z;

                    waypoints_str += std::format(";{:.7f},{:.7f},{:.2f}", lat, lon, alt);
                    waypoint_count++;
                }
            }

            payload = std::format("{:.1f}", trajectory_speed) + waypoints_str;

            // Send mission to Android.
            this->dji_.send_dji_native_trajectory(
                payload, 
                [](bool success) {
                    if (success) 
                        std::cout << "[MissionHandler] Successfully uploaded mission to Android.\n";
                    else 
                        std::cerr << "[MissionHandler] Failure to upload mission to Android.\n";
                }
            );
        }
    );
}