/**
 ******************************************************************************
 * @file    ground_station_logger.cpp
 * @brief   Implementation of the ground station logger utility.
 *
 * This file implements the QgcLogger class, encapsulating the JSON payload
 * construction for the STATUSTEXT MAVLink message.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/ground_station_logger.hpp"


/* Public Class Methods ------------------------------------------------------*/

GroundStationLogger::GroundStationLogger(mavsdk::MavlinkDirectServer& direct_server)
    : direct_server_(direct_server)
{
}

void GroundStationLogger::send_message(uint8_t severity, const std::string& message) const
{
    // text limited to 50 characters
    std::string safe_msg = message.length() > 49 ? message.substr(0, 49) : message;

    mavsdk::MavlinkDirectServer::MavlinkMessage msg;
    msg.message_name = "STATUSTEXT";
    msg.fields_json = R"({
        "severity": )" + std::to_string(severity) + R"(,
        "text": ")" + safe_msg + R"(",
        "id": 0,
        "chunk_seq": 0
    })";

    this->direct_server_.send_message(msg);
}