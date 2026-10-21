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