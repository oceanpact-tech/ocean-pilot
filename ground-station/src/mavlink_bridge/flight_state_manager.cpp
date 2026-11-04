/**
 ******************************************************************************
 * @file    flight_state_manager.cpp
 * @brief   Implementation of the thread-safe flight mode and maneuver state machine.
 *
 * This file implements the FlightStateManager class. It coordinates mode 
 * transitions requested by the Ground Control Station, handles pilot manual 
 * overrides, and provides RAII scope guards (ActionScope) for asynchronous 
 * maneuvers under a single mutex.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/flight_state_manager.hpp"

// Composed modules
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/flight_primitives.hpp"
#include "mavlink_bridge/mission_handler.hpp"
#include "mavlink_bridge/mission_executor.hpp"
#include "mavlink_bridge/ground_station_logger.hpp"
#include "mavlink_bridge/manual_control_handler.hpp"
#include "mavlink_bridge/telemetry_publisher.hpp"
#include "mavlink_bridge/maneuver_runner.hpp"

/* Public Class Methods ------------------------------------------------------*/

FlightStateManager::FlightStateManager(
    FlightActionExecutor& flight_action_executor,
    FlightPrimitives& flight_primitives,
    MissionHandler& mission_handler,
    MissionExecutor& mission_executor,
    GroundStationLogger& ground_station_logger,
    ManualControlHandler& manual_control_handler,
    TelemetryPublisher& telemetry_publisher,
    ManeuverRunner& maneuver_runner
)
    : flight_action_executor_(flight_action_executor),
      flight_primitives_(flight_primitives),
      mission_handler_(mission_handler),
      mission_executor_(mission_executor),
      ground_station_logger_(ground_station_logger),
      manual_control_handler_(manual_control_handler),
      telemetry_publisher_(telemetry_publisher),
      maneuver_runner_(maneuver_runner)
{
}

std::optional<FlightStateManager::ActionScope> FlightStateManager::try_begin_maneuver(ActiveManeuver maneuver)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (maneuver == ActiveManeuver::None || this->active_maneuver_ != ActiveManeuver::None)
        return std::nullopt;

    this->mode_before_maneuver_       = this->desired_mode_;
    this->active_maneuver_            = maneuver;
    this->phase_                      = Phase::None;
    this->want_virtual_stick_         = false; // Virtual Stick intent comes from the maneuver phase while it runs.
    this->override_during_maneuver_   = false;

    return ActionScope(this, maneuver);
}

bool FlightStateManager::request_flight_mode(FlightMode requested_mode)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    // During a maneuver only the mode to restore afterwards is updated.
    if (this->active_maneuver_ != ActiveManeuver::None)
    {
        this->mode_before_maneuver_ = requested_mode;
        return true;
    }

    if (this->desired_mode_ == requested_mode)
        return false;

    this->desired_mode_       = requested_mode;
    this->want_virtual_stick_ = (requested_mode == FlightMode::Posctl);

    return true;
}

void FlightStateManager::on_manual_override_active()
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    this->desired_mode_         = FlightMode::Hold;
    this->mode_before_maneuver_ = FlightMode::Hold;
    this->want_virtual_stick_   = false;

    // Tells finish_maneuver() not to restore a previous mode behind the pilot's back.
    if (this->active_maneuver_ != ActiveManeuver::None)
        this->override_during_maneuver_ = true;
}

FlightStateManager::FlightMode FlightStateManager::get_target_flight_mode(
    [[maybe_unused]] bool is_armed, 
    [[maybe_unused]] bool is_virtual_stick_enabled
) const
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    switch (this->active_maneuver_)
    {
        case ActiveManeuver::Takeoff:        return FlightMode::Takeoff;
        case ActiveManeuver::Land:           return FlightMode::Land;
        case ActiveManeuver::ReturnToLaunch: return FlightMode::ReturnToLaunch;
        case ActiveManeuver::Mission:        return FlightMode::Mission;
        case ActiveManeuver::Reposition:     return this->mode_before_maneuver_; // GCS keeps seeing the previous mode.
        case ActiveManeuver::None:           break;
    }

    // Posctl is reported only when the GCS asked for it. Inferring it from the 
    // hardware Virtual Stick state would feed back through the mode-change echo 
    // and re-arm the Virtual Stick intent after navigation phases.
    if (this->want_virtual_stick_)
        return FlightMode::Posctl;

    return this->desired_mode_;
}

uint8_t FlightStateManager::get_landed_state(bool is_armed) const
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->phase_ == Phase::Takeoff || this->active_maneuver_ == ActiveManeuver::Takeoff)
        return MavLandState::MAV_LANDED_STATE_TAKEOFF;

    if (this->phase_ == Phase::Landing || this->active_maneuver_ == ActiveManeuver::Land)
        return MavLandState::MAV_LANDED_STATE_LANDING;

    if (is_armed)
        return MavLandState::MAV_LANDED_STATE_IN_AIR;

    return MavLandState::MAV_LANDED_STATE_ON_GROUND;
}

bool FlightStateManager::should_virtual_stick_be_enabled(bool is_armed) const
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!is_armed)
        return false;

    if (this->active_maneuver_ == ActiveManeuver::None)
        return this->want_virtual_stick_;

    // Navigation endpoints run on top of the Virtual Stick; native takeoff, land and RTL do not.
    return this->phase_ == Phase::Navigating;
}

bool FlightStateManager::is_manual_input_allowed() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->active_maneuver_ == ActiveManeuver::None;
}

FlightStateManager::ActiveManeuver FlightStateManager::get_active_maneuver() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->active_maneuver_;
}


/* Private Class Methods -----------------------------------------------------*/

void FlightStateManager::finish_maneuver(ActiveManeuver maneuver, [[maybe_unused]] bool succeeded)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->active_maneuver_ != maneuver)
        return;

    const bool overridden = std::exchange(this->override_during_maneuver_, false);

    this->active_maneuver_ = ActiveManeuver::None;
    this->phase_           = Phase::None;

    // Reposition gives the previous mode back; every other maneuver ends in Hold.
    if (maneuver == ActiveManeuver::Reposition && !overridden)
    {
        this->desired_mode_       = this->mode_before_maneuver_;
        this->want_virtual_stick_ = (this->desired_mode_ == FlightMode::Posctl);
    }
    else
    {
        this->desired_mode_       = FlightMode::Hold;
        this->want_virtual_stick_ = false;
    }
}

void FlightStateManager::set_maneuver_phase(ActiveManeuver maneuver, Phase phase)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->active_maneuver_ == maneuver)
        this->phase_ = phase;
}

bool FlightStateManager::transition_maneuver(ActiveManeuver from, ActiveManeuver to)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->active_maneuver_ != from || to == ActiveManeuver::None)
        return false;

    this->active_maneuver_ = to;
    this->phase_           = Phase::None;

    return true;
}