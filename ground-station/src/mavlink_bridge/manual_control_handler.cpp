/**
 ******************************************************************************
 * @file    manual_control_handler.cpp
 * @brief   Implementation of DJI Virtual Stick lifecycle and joystick handling.
 *
 * This file implements the ManualControlHandler class. It manages the 
 * periodic synchronization of the DJI Virtual Stick state (including pilot 
 * manual override detection and 1.5s enable retries) and processes incoming 
 * MAVLink MANUAL_CONTROL messages at 20 Hz.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/manual_control_handler.hpp"

#include <iostream>
#include <string>

#include "mavlink_bridge/utils/mavlink_json_utils.hpp"


/* Public Class Methods ------------------------------------------------------*/

ManualControlHandler::ManualControlHandler(
    DJIInterface& dji,
    FlightStateManager& state_manager
)
    : dji_(dji),
      state_manager_(state_manager),
      last_enable_attempt_(std::chrono::steady_clock::now() - std::chrono::seconds(5)),
      last_stick_send_(std::chrono::steady_clock::now())
{
}

void ManualControlHandler::maintain_virtual_stick()
{
    const bool is_virtual_stick_enabled = this->dji_.is_virtual_stick_enabled();
    const bool is_armed                 = this->dji_.is_armed();

    // If pilot takes control back via manual override, drop Posctl and revert to Hold.
    if (this->dji_.is_manual_override_active())
    {
        this->state_manager_.on_manual_override_active();
        if (is_virtual_stick_enabled)
            this->dji_.disable_virtual_stick();
        return;
    }

    // Disable Virtual Stick if not wanted by current state or drone is disarmed.
    if (!this->state_manager_.should_virtual_stick_be_enabled(is_armed))
    {
        if (is_virtual_stick_enabled)
            this->dji_.disable_virtual_stick();
        return;
    }

    // Retry enabling Virtual Stick every 1.5 seconds if required but not yet active.
    if (!is_virtual_stick_enabled)
    {
        auto now = std::chrono::steady_clock::now();

        if (!this->is_virtual_stick_enabling_ && 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_enable_attempt_).count() >= 1500)
        {
            this->is_virtual_stick_enabling_ = true;
            this->last_enable_attempt_       = now;

            this->dji_.enable_virtual_stick([this](bool success) {
                this->is_virtual_stick_enabling_ = false;
                if (!success) {
                    std::cerr << "[ManualControlHandler] Falha HTTP ao enviar enableVirtualStick\n";
                }
            });
        }
    }
}

bool ManualControlHandler::handle_manual_control_message(const mavsdk::Mavsdk::MavlinkMessage& msg)
{
    if (!this->dji_.is_virtual_stick_enabled())
        return true;

    auto lx_str = extract_json_value(msg.fields_json, "r"); // Yaw      (-1000 to 1000)
    auto ly_str = extract_json_value(msg.fields_json, "z"); // Throttle (0 to 1000)
    auto rx_str = extract_json_value(msg.fields_json, "y"); // Roll     (-1000 to 1000)
    auto ry_str = extract_json_value(msg.fields_json, "x"); // Pitch    (-1000 to 1000)

    if (rx_str && ry_str && lx_str && ly_str)
    {
        float lx = std::stof(lx_str.value());
        float ly = std::stof(ly_str.value());
        float rx = std::stof(rx_str.value());
        float ry = std::stof(ry_str.value());

        float div = this->stick_scale_divider_.load();
        if (div < 0.1f)
            div = 1.0f;

        lx = (lx / 1000.f) / div;
        ly = ((ly - 500.f) / 500.f) / div;
        rx = (rx / 1000.f) / div;
        ry = (ry / 1000.f) / div;

        std::string payload = std::to_string(lx) + "," +
                              std::to_string(ly) + "," +
                              std::to_string(rx) + "," +
                              std::to_string(ry);

        auto now = std::chrono::steady_clock::now();

        // Send at 20 Hz (every 50ms)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_stick_send_).count() >= 50)
        {
            this->dji_.send_stick(payload);
            this->last_stick_send_ = now;
        }

        return true;
    }

    return false;
}

void ManualControlHandler::set_stick_scale_divider(float divider) noexcept
{
    if (divider >= 0.1f)
        this->stick_scale_divider_.store(divider);
}