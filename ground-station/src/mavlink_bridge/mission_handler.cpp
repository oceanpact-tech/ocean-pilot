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
#include "mavlink_bridge/flight_action_executor.hpp"

/* Public Class Methods ------------------------------------------------------*/

MissionHandler::MissionHandler(
    DJIInterface& dji, 
    mavsdk::MissionRawServer& mission_server,
    FlightActionExecutor& flight_action_executor
)
    : dji_(dji), 
      mission_server_(mission_server),
      flight_action_executor_(flight_action_executor)
{
}

MissionHandler::~MissionHandler() 
{
    this->mission_server_.unsubscribe_incoming_mission(this->incoming_mission_handle_);
}

void MissionHandler::start() 
{
    this->incoming_mission_handle_ = this->mission_server_.subscribe_incoming_mission(
        [this](Result result, MissionPlan plan) 
        {
            return this->incoming_mission_callback(result, plan);
        }
    );
}

void MissionHandler::incoming_mission_callback(Result result, MissionPlan plan)
{
    if (result != Result::Success) 
    {
        std::cerr << "[MissionHandler] Failure to receive mission from QGC: " << result << '\n';
        return;
    }

    if (plan.mission_items.empty()) 
        return;

    std::cerr << "[MissionHandler] Succesfully received mission from QGC\n";
    this->flight_action_executor_.save_new_mission_plan(plan);
}