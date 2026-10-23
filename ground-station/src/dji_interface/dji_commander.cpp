/**
 ******************************************************************************
 * @file    DJICommander.cpp
 * @brief   Implementation of the DJICommander class.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


/* Private Includes ----------------------------------------------------------*/

#include "dji_interface/dji_commander.hpp"


/* Public Class Methods ------------------------------------------------------*/

void DJICommander::request_takeoff(std::function<void(bool)> on_complete) 
{
    this->http_client_.post_async(std::string(endpoints::post::TAKEOFF), "", on_complete);
}

void DJICommander::request_land(std::function<void(bool)> on_complete) 
{
    this->http_client_.post_async(std::string(endpoints::post::LAND), "", on_complete);
}

void DJICommander::request_return_to_home(std::function<void(bool)> on_complete) 
{
    this->http_client_.post_async(std::string(endpoints::post::RETURN_TO_HOME), "", on_complete);
}

void DJICommander::enable_virtual_stick(std::function<void(bool)> on_complete) 
{
    this->http_client_.post_async(std::string(endpoints::post::ENABLE_VIRTUAL_STICK), "true", on_complete);
}

void DJICommander::disable_virtual_stick(std::function<void(bool)> on_complete) 
{
    this->http_client_.post_async(std::string(endpoints::post::ENABLE_VIRTUAL_STICK), "false", on_complete);
}

void DJICommander::send_stick(std::string payload, std::function<void(bool)> on_complete) 
{
    this->http_client_.post_async(std::string(endpoints::post::STICK), payload, on_complete);
}

void DJICommander::go_to_altitude(float altitude, std::function<void(bool)> on_complete) 
{
    std::string payload = std::to_string(altitude);
    this->http_client_.post_async(std::string(endpoints::post::GO_TO_ALTITUDE), payload, on_complete);
}

void DJICommander::go_to_altitude_by_position(float altitude, std::function<void(bool)> on_complete) 
{
    std::string payload = std::to_string(altitude);
    this->http_client_.post_async(std::string(endpoints::post::GO_TO_ALTITUDE_BY_POSITION), payload, on_complete);
}

void DJICommander::send_dji_native_trajectory(std::string payload, std::function<void(bool)> on_complete) 
{
    this->http_client_.post_async(std::string(endpoints::post::GO_TO_TRAJECTORY_DJI_NATIVE), payload, on_complete);
}