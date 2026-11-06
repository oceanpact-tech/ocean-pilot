/**
 ******************************************************************************
 * @file    maneuver_runner.cpp
 * @brief   Implementation of the single maneuver worker thread.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/maneuver_runner.hpp"

#include <iostream>
#include <utility>

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

ManeuverRunner::ManeuverRunner(
    FlightStateManager& state_manager,
    FlightActionExecutor& flight_action_executor,
    FlightPrimitives& flight_primitives,
    MissionHandler& mission_handler,
    MissionExecutor& mission_executor,
    GroundStationLogger& ground_station_logger,
    ManualControlHandler& manual_control_handler,
    TelemetryPublisher& telemetry_publisher
)
    : state_manager_(state_manager),
      flight_primitives_(flight_primitives),
      mission_handler_(mission_handler),
      mission_executor_(mission_executor),
      flight_action_executor_(flight_action_executor),
      ground_station_logger_(ground_station_logger),
      manual_control_handler_(manual_control_handler),
      telemetry_publisher_(telemetry_publisher)
{
}

ManeuverRunner::~ManeuverRunner()
{
    this->cancel();
}

bool ManeuverRunner::run(Maneuver maneuver, Policy policy, Body body)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (policy == Policy::PreemptCurrent && this->worker_.joinable())
    {
        // The unwinding body releases its scope (and restores the state) before join() returns.
        this->worker_.request_stop();
        this->worker_.join();
    }

    auto scope = this->state_manager_.try_begin_maneuver(maneuver);
    if (!scope)
        return false;

    // The previous worker (if any) already released its scope: reclaim the thread.
    if (this->worker_.joinable())
        this->worker_.join();

    this->worker_ = std::jthread(
        [scope = std::move(*scope), body = std::move(body)](std::stop_token stoken) mutable
        {
            // Moved to a local so the scope is released when the body returns, 
            // independently of when the thread state is destroyed.
            FlightStateManager::ActionScope local_scope(std::move(scope));

            try
            {
                body(stoken, local_scope);
            }
            catch (const std::exception& e)
            {
                std::cerr << "[ManeuverRunner] Unhandled exception in maneuver: " << e.what() << '\n';
            }
            catch (...)
            {
                std::cerr << "[ManeuverRunner] Unhandled unknown exception in maneuver\n";
            }
        }
    );

    return true;
}

void ManeuverRunner::cancel()
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (this->worker_.joinable())
    {
        this->worker_.request_stop();
        this->worker_.join();
    }
}

bool ManeuverRunner::is_busy() const
{
    return this->state_manager_.get_active_maneuver() != Maneuver::None;
}