/**
 ******************************************************************************
 * @file    dji_interface.cpp
 * @brief   Implementation of the DJIInterface class.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


/* Private Includes ----------------------------------------------------------*/

#include "dji_interface/dji_interface.hpp"


/* Public Class Methods ------------------------------------------------------*/

/* @brief Stops telemetry stream
 */
DJIInterface::~DJIInterface() {
    this->stop_telemetry_stream();
}

void DJIInterface::start_telemetry_stream() 
{
    this->telemetry_client_.start_comm();
}

void DJIInterface::stop_telemetry_stream() 
{
    this->telemetry_client_.stop_comm();
}

void DJIInterface::request_takeoff(std::function<void()> on_complete) {
    // Converte a string_view do namespace para std::string e envia vazio no payload
    this->http_client_.post_async(std::string(endpoints::post::TAKEOFF), "", on_complete);
}


/* Private Class Methods -----------------------------------------------------*/