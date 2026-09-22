/**
 ******************************************************************************
 * @file    wild_bridge_endpoints.hpp
 * @brief   Definitions of HTTP endpoints for the DJI WildBridge API.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */

#pragma once


/* Includes ------------------------------------------------------------------*/

#include <string_view>


/* Namespaces ----------------------------------------------------------------*/

/**
 * @brief Group of endpoints for communication with the DJI WildBridge API.
 */
namespace endpoints 
{
    /**
     * @brief HTTP POST Command Endpoints (port 8080).
     * Used as setters to trigger aircraft actions or configure parameters.
     */
    namespace post 
    {
        // constexpr std::string_view STICK = "/send/stick";  // expects a formatted string: "<leftX>,<leftY>,<rightX>,<rightY>"
        // constexpr std::string_view ZOOM = "/send/camera/zoom";
        // constexpr std::string_view GIMBAL_SET_PITCH = "/send/gimbal/pitch";
        // constexpr std::string_view GIMBAL_SET_YAW = "/send/gimbal/yaw";  // !!! This is the yaw joint angle !!!
        // constexpr std::string_view GIMBAL_SET_REL_PITCH = "/send/gimbal/rel_pitch";
        // constexpr std::string_view GIMBAL_SET_REL_YAW = "/send/gimbal/rel_yaw";
        constexpr std::string_view TAKEOFF = "/send/takeoff";
        // constexpr std::string_view LAND = "/send/land";
        // constexpr std::string_view RTH = "/send/RTH";
        // constexpr std::string_view ENABLE_VIRTUAL_STICK = "/send/enableVirtualStick";
        // constexpr std::string_view ABORT_MISSION = "/send/abortMission";
        // constexpr std::string_view ABORT_ALL = "/send/abortAll";
        // constexpr std::string_view GOTO_YAW = "/send/gotoYaw";
        // constexpr std::string_view GOTO_WP_NOSE_FORWARD = "/send/gotoWaypointNoseForward";
        // constexpr std::string_view GOTO_ALTITUDE = "/send/gotoAltitude";
        // constexpr std::string_view CAMERA_START_RECORDING = "/send/camera/startRecording";
        // constexpr std::string_view CAMERA_STOP_RECORDING = "/send/camera/stopRecording";
        // constexpr std::string_view GOTO_TRAJECTORY_DJI_NATIVE = "/send/navigateTrajectoryDJINative";
        // constexpr std::string_view ABORT_DJI_NATIVE_MISSION = "/send/abort/DJIMission";
        // constexpr std::string_view SET_RTH_ALTITUDE = "/send/setRTHAltitude";
        // constexpr std::string_view DEACTIVATE_MANUAL_OVERRIDE = "/send/deactivateManualOverride";
        // constexpr std::string_view GET_MANUAL_OVERRIDE = "/get/isManualOverrideActive";
        // constexpr std::string_view LRF_MEASURE = "/send/lrf/measure";
        // constexpr std::string_view LIST_MEDIA = "/send/listMedia";
        // constexpr std::string_view DOWNLOAD_MEDIA_BY_NAME = "/send/downloadMediaByName";

        /// PID Tuning
        // constexpr std::string_view GOTO_WP_HOLD_HEADING = "/send/gotoWaypointHoldHeading";
        // constexpr std::string_view PAYLOAD_DROP = "/send/drop";
    }

    /**
     * @brief Aircraft state endpoint suffixes for HTTP GET requests.
     * Used to retrieve telemetry, positioning, and subsystem statuses.
     */
    namespace get 
    {
        // constexpr std::string_view BASE = "/";
        // constexpr std::string_view SPEED = "/aircraft/speed";
        // constexpr std::string_view HEADING = "/aircraft/heading";
        // constexpr std::string_view ATTITUDE = "/aircraft/attitude";
        // constexpr std::string_view LOCATION = "/aircraft/location";
        // constexpr std::string_view GIMBAL_ATTITUDE = "/aircraft/gimbalAttitude";
        // constexpr std::string_view ALL_STATES = "/aircraft/allStates";
        // constexpr std::string_view STICK_VALUES = "/aircraft/rcStickValues";
        // constexpr std::string_view YAW_REACHED = "/status/yawReached";
        // constexpr std::string_view ALTITUDE_REACHED = "/status/altitudeReached";
        // constexpr std::string_view WP_REACHED = "/status/waypointReached";
        // constexpr std::string_view HOME_LOCATION = "/home/location";
        // constexpr std::string_view CAMERA_IS_RECORDING = "/status/camera/isRecording";
        // constexpr std::string_view CAPTURE_THERMAL_IMAGE = "/send/captureThermalImage";
        // constexpr std::string_view CAPTURE_TEMPERATURE = "/send/captureTemperature";  // temperature-only read, no shutter
    }
}