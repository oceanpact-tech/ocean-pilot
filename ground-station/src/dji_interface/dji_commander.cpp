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

void DJICommander::request_arm(std::function<void()> on_complete) {
    this->http_client_.post_async(std::string(endpoints::post::ARM), "", on_complete);
}

void DJICommander::request_disarm(std::function<void()> on_complete) {
    this->http_client_.post_async(std::string(endpoints::post::DISARM), "", on_complete);
}

void DJICommander::request_takeoff(std::function<void()> on_complete) {
    this->http_client_.post_async(std::string(endpoints::post::TAKEOFF), "", on_complete);
}

/* Private Class Methods -----------------------------------------------------*/