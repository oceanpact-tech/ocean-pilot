/**
 ******************************************************************************
 * @file    flight_primitives.cpp
 * @brief   Implementation of the blocking and cancellable flight operations.
 *
 * This file implements the FlightPrimitives class. Every operation follows the 
 * same pattern: check preconditions, publish the phase, wait for the Virtual 
 * Stick to reach the required state, send the command, confirm it started and 
 * wait for completion with a time budget.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "mavlink_bridge/flight_primitives.hpp"

#include <algorithm>
#include <atomic>
#include <format>
#include <memory>
#include <thread>

// Composed modules
#include "mavlink_bridge/flight_state_manager.hpp"
#include "mavlink_bridge/flight_primitives.hpp"
#include "mavlink_bridge/mission_handler.hpp"
#include "mavlink_bridge/mission_executor.hpp"
#include "mavlink_bridge/ground_station_logger.hpp"
#include "mavlink_bridge/manual_control_handler.hpp"
#include "mavlink_bridge/telemetry_publisher.hpp"
#include "mavlink_bridge/maneuver_runner.hpp"

// Helpers
#include "mavlink_bridge/utils/http_response_utils.hpp"


/* Aliases -------------------------------------------------------------------*/

using Phase = FlightStateManager::Phase;


/* Local helpers -------------------------------------------------------------*/

namespace
{
    /// @brief State of one HTTP request. Shared because the callback may fire after the caller returned.
    struct HttpResult 
    {
        std::atomic<bool> done{false};    ///< Set after the callback stored its result.
        std::atomic<bool> success{false}; ///< HTTP success flag.
        std::string       response;       ///< Response body, written before `done` is set.
    };

    constexpr const char* STATUS_IDLE       = "IDLE";
    constexpr const char* STATUS_HOVERING   = "HOVERING";
    constexpr const char* STATUS_NAVIGATING = "NAVIGATING";
}


/* Public Class Methods ------------------------------------------------------*/

FlightPrimitives::FlightPrimitives(
    // DJI API
    DJIInterface& dji,
    // Composed modules
    FlightStateManager& state_manager,
    FlightActionExecutor& flight_action_executor,
    MissionHandler& mission_handler,
    MissionExecutor& mission_executor,
    GroundStationLogger& ground_station_logger,
    ManualControlHandler& manual_control_handler,
    TelemetryPublisher& telemetry_publisher,
    ManeuverRunner& maneuver_runner,
    // General attributes
    PrimitiveTimeouts timeouts
)
    : // DJI API
      dji_(dji),
      // Composed modules
      state_manager_(state_manager),
      ground_station_logger_(ground_station_logger),
      mission_handler_(mission_handler),
      mission_executor_(mission_executor),
      flight_action_executor_(flight_action_executor),
      manual_control_handler_(manual_control_handler),
      telemetry_publisher_(telemetry_publisher),
      maneuver_runner_(maneuver_runner),
      // General attributes
      timeouts_(timeouts)
{
}

Outcome FlightPrimitives::takeoff(Scope& scope, std::stop_token stoken, const TickFn& on_tick)
{
    if (!this->dji_.is_ready_to_takeoff())
        return Outcome::Failed;

    scope.set_phase(Phase::Takeoff);

    if (const Outcome o = this->wait_virtual_stick(false, stoken, on_tick); o != Outcome::Done)
        return o;

    const Outcome started = this->send_and_confirm_start(
        [this](std::function<void(bool)> cb) { this->dji_.request_takeoff(std::move(cb)); },
        "TAKING_OFF", stoken, on_tick
    );
    if (started != Outcome::Done)
        return started;

    return this->wait_until(
        [this]() { return this->dji_.get_drone_status() == STATUS_IDLE; },
        this->timeouts_.takeoff, stoken, on_tick
    );
}

Outcome FlightPrimitives::land(Scope& scope, std::stop_token stoken, const TickFn& on_tick)
{
    if (!this->dji_.is_armed())
        return Outcome::Failed;

    scope.set_phase(Phase::Landing);

    if (const Outcome o = this->wait_virtual_stick(false, stoken, on_tick); o != Outcome::Done)
        return o;

    const Outcome started = this->send_and_confirm_start(
        [this](std::function<void(bool)> cb) { this->dji_.request_land(std::move(cb)); },
        "LANDING", stoken, on_tick
    );
    if (started != Outcome::Done)
        return started;

    return this->wait_until(
        [this]() { return this->dji_.get_drone_status() == STATUS_IDLE || !this->dji_.is_armed(); },
        this->timeouts_.land, stoken, on_tick
    );
}

Outcome FlightPrimitives::return_to_launch(Scope& scope, std::stop_token stoken, const TickFn& on_tick)
{
    if (!this->dji_.is_armed())
        return Outcome::Failed;

    scope.set_phase(Phase::ReturnToLaunch);

    if (const Outcome o = this->wait_virtual_stick(false, stoken, on_tick); o != Outcome::Done)
        return o;

    const Outcome started = this->send_and_confirm_start(
        [this](std::function<void(bool)> cb) { this->dji_.request_return_to_home(std::move(cb)); },
        "RETURNING_HOME", stoken, on_tick
    );
    if (started != Outcome::Done)
        return started;

    return this->wait_until(
        [this]() { return this->dji_.get_drone_status() == STATUS_IDLE || !this->dji_.is_armed(); },
        this->timeouts_.return_to_launch, stoken, on_tick
    );
}

Outcome FlightPrimitives::change_altitude(Scope& scope, float altitude_rel_m, std::stop_token stoken, const TickFn& on_tick)
{
    if (!this->dji_.is_armed())
        return Outcome::Failed;

    // The app does not accept targets below 1 m.
    altitude_rel_m = std::max(altitude_rel_m, 1.0f);

    scope.set_phase(Phase::Navigating);

    if (const Outcome o = this->wait_virtual_stick(true, stoken, on_tick); o != Outcome::Done)
        return o;

    const Outcome started = this->send_and_confirm_start(
        [this, altitude_rel_m](std::function<void(bool)> cb) {
            this->dji_.go_to_altitude_by_position(altitude_rel_m, std::move(cb));
        },
        STATUS_NAVIGATING, stoken, on_tick
    );
    if (started != Outcome::Done)
        return started;

    const Outcome arrived = this->wait_until(
        [this]() {
            const auto status = this->dji_.get_drone_status();
            return status == STATUS_IDLE || status == STATUS_HOVERING;
        },
        this->timeouts_.change_altitude, stoken, on_tick
    );
    if (arrived != Outcome::Done)
        return arrived;

    return this->dji_.is_armed() ? Outcome::Done : Outcome::Failed;
}

Outcome FlightPrimitives::go_to_waypoint(Scope& scope, const WaypointCommand& waypoint, std::stop_token stoken, const TickFn& on_tick)
{
    if (!this->dji_.is_armed())
        return Outcome::Failed;

    scope.set_phase(Phase::Navigating);

    if (const Outcome o = this->wait_virtual_stick(true, stoken, on_tick); o != Outcome::Done)
        return o;

    const int attempts = std::max(waypoint.max_attempts, 1);

    for (int attempt = 0; attempt < attempts; attempt++)
    {
        auto http = std::make_shared<HttpResult>();

        this->send_waypoint(waypoint, [http](bool success, std::string response) {
            http->response = std::move(response);
            http->success  = success;
            http->done     = true;
        });

        // 1. Wait for the drone to start navigating (or for the HTTP request to fail).
        const Outcome started = this->wait_until(
            [this, http]() {
                return (http->done && !http->success) || this->dji_.get_drone_status() == STATUS_NAVIGATING;
            },
            this->timeouts_.start, stoken, on_tick
        );

        if (started == Outcome::Cancelled || started == Outcome::Overridden)
            return started;

        if (started == Outcome::Timeout || (http->done && !http->success))
            continue; // Not accepted: send the leg again.

        // 2. Wait for the leg to end.
        const Outcome arrived = this->wait_until(
            [this]() { return this->dji_.get_drone_status() == STATUS_IDLE; },
            this->timeouts_.waypoint, stoken, on_tick
        );
        if (arrived != Outcome::Done)
            return arrived;

        // 3. The response carries the id used to confirm the arrival.
        if (!http->done)
        {
            const Outcome response = this->wait_until(
                [http]() { return http->done.load(); },
                this->timeouts_.start, stoken, on_tick
            );
            if (response == Outcome::Cancelled || response == Outcome::Overridden)
                return response;
            if (response == Outcome::Timeout)
                continue;
        }

        const int sequence_id = extract_waypoint_seq(http->response);

        if (this->dji_.is_waypoint_reached(sequence_id))
            return this->dji_.is_armed() ? Outcome::Done : Outcome::Failed;

        // Not confirmed as reached: send the leg again.
    }

    return Outcome::Failed;
}

Outcome FlightPrimitives::hold(std::chrono::milliseconds duration, std::stop_token stoken)
{
    const auto deadline = std::chrono::steady_clock::now() + duration;

    while (true)
    {
        if (const auto aborted = this->check_abort(stoken))
            return *aborted;

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
            return Outcome::Done;

        std::this_thread::sleep_for(std::min(
            this->timeouts_.poll_interval,
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
        ));
    }
}

Outcome FlightPrimitives::brake(Scope& scope, const TickFn& on_tick)
{
    const std::stop_token never_stops; // Cleanup of a cancelled maneuver: ignore cancellation.

    scope.set_phase(Phase::Navigating);

    if (const Outcome o = this->wait_virtual_stick(true, never_stops, on_tick); o != Outcome::Done)
        return o;

    WaypointCommand here;
    here.target       = this->dji_.get_location();
    here.yaw_deg      = static_cast<float>(this->dji_.get_attitude().yaw);
    here.heading_mode = HeadingMode::HoldHeading;
    here.max_attempts = 1;

    auto http = std::make_shared<HttpResult>();
    this->send_waypoint(here, [http](bool success, std::string response) {
        http->response = std::move(response);
        http->success  = success;
        http->done     = true;
    });

    const Outcome started = this->wait_until(
        [this, http]() {
            return (http->done && !http->success) || this->dji_.get_drone_status() == STATUS_NAVIGATING;
        },
        std::chrono::seconds(3), never_stops, on_tick
    );

    if (started == Outcome::Overridden)
        return started;

    if (http->done && !http->success)
        return Outcome::Failed;

    // The go-to may be so short that NAVIGATING is never observed: the drone is already stopped.
    if (started == Outcome::Timeout)
        return Outcome::Done;

    return this->wait_until(
        [this]() { return this->dji_.get_drone_status() == STATUS_IDLE; },
        this->timeouts_.brake, never_stops, on_tick
    );
}


/* Private Class Methods -----------------------------------------------------*/

std::optional<Outcome> FlightPrimitives::check_abort(const std::stop_token& stoken) const
{
    // Never command the drone against the pilot.
    if (this->dji_.is_manual_override_active())
        return Outcome::Overridden;

    if (stoken.stop_requested())
        return Outcome::Cancelled;

    return std::nullopt;
}

Outcome FlightPrimitives::wait_until(
    const std::function<bool()>& predicate,
    std::chrono::seconds timeout,
    const std::stop_token& stoken,
    const TickFn& on_tick
) const
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (true)
    {
        if (const auto aborted = this->check_abort(stoken))
            return *aborted;

        if (predicate())
            return Outcome::Done;

        if (std::chrono::steady_clock::now() >= deadline)
            return Outcome::Timeout;

        if (on_tick)
            on_tick();

        std::this_thread::sleep_for(this->timeouts_.poll_interval);
    }
}

Outcome FlightPrimitives::wait_virtual_stick(bool enabled, const std::stop_token& stoken, const TickFn& on_tick) const
{
    return this->wait_until(
        [this, enabled]() { return this->dji_.is_virtual_stick_enabled() == enabled; },
        this->timeouts_.virtual_stick, stoken, on_tick
    );
}

Outcome FlightPrimitives::send_and_confirm_start(
    const HttpSender& send,
    std::string_view started_status,
    const std::stop_token& stoken,
    const TickFn& on_tick
) const
{
    auto http = std::make_shared<HttpResult>();

    send([http](bool success) {
        http->success = success;
        http->done    = true;
    });

    const Outcome waited = this->wait_until(
        [this, http, started_status]() {
            return (http->done && !http->success) || this->dji_.get_drone_status() == started_status;
        },
        this->timeouts_.start, stoken, on_tick
    );

    if (waited == Outcome::Timeout)
        return Outcome::Failed; // The drone never reported the operation started.

    if (waited != Outcome::Done)
        return waited;

    return (http->done && !http->success) ? Outcome::Failed : Outcome::Done;
}

void FlightPrimitives::send_waypoint(
    const WaypointCommand& waypoint,
    std::function<void(bool, std::string)> on_complete
) const
{
    const float yaw = waypoint.yaw_deg.value_or(static_cast<float>(this->dji_.get_attitude().yaw));

    // payload: "lat,lon,alt,yaw[,speed]"
    std::string payload = std::format(
        "{:.7f},{:.7f},{:.2f},{:.1f}",
        waypoint.target.latitude, waypoint.target.longitude, waypoint.target.altitude, yaw
    );

    if (waypoint.speed_m_s)
        payload += std::format(",{:.1f}", *waypoint.speed_m_s);

    switch (waypoint.heading_mode)
    {
        case HeadingMode::NoseForward:
            this->dji_.go_to_waypoint_nose_forward(payload, std::move(on_complete));
            break;

        case HeadingMode::HoldHeading:
            this->dji_.go_to_waypoint_hold_heading(payload, std::move(on_complete));
            break;
    }
}