/**
 ******************************************************************************
 * @file    flight_state_manager.cpp
 * @brief   Implementation of the thread-safe flight mode and maneuver state machine.
 *
 * This file implements the FlightStateManager class. It coordinates mode 
 * transitions requested by the Ground Control Station, handles pilot manual 
 * overrides, and provides RAII scope guards (ActionScope) for asynchronous 
 * Takeoff and Land maneuvers under a single mutex.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/flight_action_executor.hpp"


/* Public Class Methods ------------------------------------------------------*/

std::optional<FlightStateManager::ActionScope> FlightStateManager::try_begin_maneuver(ActiveManeuver maneuver)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (maneuver == ActiveManeuver::None || this->active_maneuver_ != ActiveManeuver::None)
        return std::nullopt;

    const FlightMode mode_before = this->desired_mode_;

    this->active_maneuver_    = maneuver;
    this->want_virtual_stick_ = false; // Always disable Virtual Stick during autonomous maneuvers.

    return ActionScope(this, maneuver, mode_before);
}

bool FlightStateManager::request_flight_mode(FlightMode requested_mode)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->desired_mode_ == requested_mode && this->active_maneuver_ == ActiveManeuver::None)
        return false;

    this->desired_mode_       = requested_mode;
    this->want_virtual_stick_ = (requested_mode == FlightMode::Posctl);

    return true;
}

void FlightStateManager::on_manual_override_active()
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    this->desired_mode_       = FlightMode::Hold;
    this->want_virtual_stick_ = false;
}

FlightStateManager::FlightMode FlightStateManager::get_target_flight_mode(
    bool is_armed, 
    bool is_virtual_stick_enabled
) const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    bool is_executing_mission = this->flight_action_executor_.is_executing_mission();

    if (is_executing_mission)
        return FlightMode::Mission;

    if (this->active_maneuver_ == ActiveManeuver::Takeoff)
        return FlightMode::Takeoff;

    if (this->active_maneuver_ == ActiveManeuver::Land)
        return FlightMode::Land;
    
    if (this->active_maneuver_ == ActiveManeuver::ReturnToLaunch)
        return FlightMode::ReturnToLaunch;

    if (this->want_virtual_stick_ || (is_virtual_stick_enabled && is_armed))
        return FlightMode::Posctl;

    return this->desired_mode_;
}

uint8_t FlightStateManager::get_landed_state(bool is_armed) const
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->active_maneuver_ == ActiveManeuver::Takeoff)
        return MavLandState::MAV_LANDED_STATE_TAKEOFF;

    if (this->active_maneuver_ == ActiveManeuver::Land)
        return MavLandState::MAV_LANDED_STATE_LANDING;

    if (is_armed)
        return MavLandState::MAV_LANDED_STATE_IN_AIR;

    return MavLandState::MAV_LANDED_STATE_ON_GROUND;
}

bool FlightStateManager::should_virtual_stick_be_enabled(bool is_armed) const
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    bool is_executing_mission = this->flight_action_executor_.is_executing_mission();

    return (this->want_virtual_stick_ || is_executing_mission) && 
           is_armed && 
           (this->active_maneuver_ == ActiveManeuver::None);
}

FlightStateManager::ActiveManeuver FlightStateManager::get_active_maneuver() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->active_maneuver_;
}


/* Private Class Methods -----------------------------------------------------*/

void FlightStateManager::finish_maneuver(ActiveManeuver maneuver, FlightMode mode_before, bool succeeded)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->active_maneuver_ != maneuver)
        return;

    this->active_maneuver_ = ActiveManeuver::None;

    // Drone MUST engage Hold mode when autonomous actions completes.
    this->desired_mode_       = FlightMode::Hold;
    this->want_virtual_stick_ = false;
}