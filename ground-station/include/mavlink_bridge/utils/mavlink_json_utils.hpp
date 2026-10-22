/**
 ******************************************************************************
 * @file    mavlink_json_utils.hpp
 * @brief   Stateless JSON and MAVLink message helper functions.
 *
 * This file provides utility functions for constructing raw MAVLink JSON 
 * payloads (such as custom COMMAND_ACK messages) and extracting key-value 
 * pairs from incoming and outgoing MAVSDK JSON message strings without 
 * external parser dependencies.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once

/* Includes ------------------------------------------------------------------*/

#include <string>
#include <optional>
#include <cstdint>

// MAVSDK
#include <mavsdk/plugins/mavlink_direct_server/mavlink_direct_server.hpp>


/* Functions  ----------------------------------------------------------------*/

/**
 * @brief Create a formatted mavlink message to be sent as COMMAND_ACK.
 * 
 * Constructs a COMMAND_ACK message tagged with a custom magic number (999) 
 * in result_param2 so the outgoing message filter can distinguish manual ACKs 
 * from automatic MAVSDK ACKs.
 * 
 * @param command The MAV_CMD identifier being acknowledged.
 * @param result  The MAV_RESULT status code (e.g., ACCEPTED, DENIED, IN_PROGRESS).
 * @return Formatted MavlinkMessage ready to be dispatched via MavlinkDirectServer.
 */
inline mavsdk::MavlinkDirectServer::MavlinkMessage create_ack_mavlink_msg(uint16_t command, uint8_t result)
{
    mavsdk::MavlinkDirectServer::MavlinkMessage msg;
    msg.message_name = "COMMAND_ACK";
    msg.fields_json  = R"({
        "command":)" + std::to_string(command) + R"(,
        "result":)" + std::to_string(result) + R"(,
        "progress":255,
        "result_param2":999,
        "target_component":190,
        "target_system":255
    })";
    return msg;
}

/**
 * @brief Get the value for a specific key from a JSON string.
 * 
 * Performs a lightweight string search to extract string, numeric, or boolean 
 * values associated with a given key in a flat JSON payload.
 * 
 * @param json_str The raw JSON string to parse.
 * @param key      The target key name to search for.
 * @return std::optional<std::string> containing the extracted value, or std::nullopt if not found.
 */
inline std::optional<std::string> extract_json_value(const std::string& json_str, const std::string& key) 
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