/**
 ******************************************************************************
 * @file    test_telemetry.cpp
 * @brief   Test file for the telemetry functionality.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "dji_interface/dji_interface.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/signal_set.hpp>


/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;


/* Definitions ---------------------------------------------------------------*/
#define GET_ALL                            0
#define GET_DRONE_NAME                     0
#define GET_SPEED                          1
#define GET_HEADING                        0
#define GET_ATTITUDE                       0
#define GET_LOCATION                       0
#define GET_GIMBAL_ATTITUDE                0
#define GET_GIMBAL_JOINT_ATTITUDE          0
#define GET_ZOOM_FOCAL_LENGTH              0
#define GET_HYBRID_FOCAL_LENGTH            0
#define GET_OPTICAL_FOCAL_LENGTH           0
#define GET_ZOOM_RATIO                     0
#define GET_BATTERY_LEVEL                  0
#define GET_SATTELITE_COUNT                0
#define IS_READY_TO_TAKEOFF                0
#define GET_TAKEOFF_BLOCK_REASON           0
#define GET_HOME_LOCATION                  0
#define GET_DISTANCE_TO_HOME               0
#define GET_LRF_TARGET                     0
#define GET_WAYPOINT_SEQ                   0
#define IS_WAYPOINT_REACHED                0
#define IS_INTERMEDIARY_WAYPOINT_REACHED   0
#define GET_YAW_SEQ                        0
#define IS_YAW_REACHED                     0
#define GET_ALTITUDE_SEQ                   0 
#define IS_ALTITUDE_REACHED                0
#define IS_CAMERA_RECORDING                0
#define IS_HOME_SET                        0
#define GET_REMAINING_FLIGHT_TIME          0
#define GET_TIME_NEEDED_TO_GO_HOME         0
#define GET_TIME_NEEDED_TO_LAND            0
#define GET_TOTAL_TIME                     0
#define GET_MAX_RADIUS_CAN_FLY_AND_GO_HOME 0
#define GET_REMAINING_CHARGE               0
#define GET_BATTERY_NEEDED_TO_LAND         0
#define GET_BATTERY_NEEDED_TO_GO_HOME      0
#define GET_SERIOUS_LOW_BATTERY_THRESHOLD  0
#define GET_LOW_BATTERY_THRESHOLD          0
#define GET_FLIGHT_MODE                    0
#define IS_MANUAL_OVERRIDE_ACTIVE          0


/* Entrypoint ----------------------------------------------------------------*/

int main(int argc, char* argv[]) 
{
    // The io_context is required for all I/O
    net::io_context ioc;
    auto work_guard = net::make_work_guard(ioc);

    // Connect to the drone using the DJIInterface
    DJIInterface dji(ioc, "192.168.228.24", true, true);

    // Set up signal handling for graceful shutdown
    net::signal_set signals(ioc, SIGINT, SIGTERM);

    // Book closing coroutine
    signals.async_wait([&](const boost::system::error_code& error, int signal_number) {
        if (!error) 
        {
            std::cout << "\n[System] Interruption detected (Signal " << signal_number << ").\n";
            std::cout << "[System] Starting graceful shutdown sequence...\n";
            
            // Stop telemetry stream and clean up
            dji.stop_telemetry_stream();
            work_guard.reset(); 
            ioc.stop();
        }
    });

    // Run the IO service in background.
    std::thread asio_thread([&ioc]() {
        ioc.run();
    });
    std::cout << "[DEBUG] System initialized.\n";

    dji.start_telemetry_stream();
    std::cout << "[DEBUG] Telemetry stream started.\n";

    // Stablish connection with the telemetry stream
    while (!dji.is_connected() && !ioc.stopped())
    {
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);

        std::cout << "[DEBUG - " << std::put_time(std::localtime(&current_time), "%H:%M:%S") << "] Waiting for telemetry stream to connect...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Control loop
    std::cout << "[DEBUG] Starting control loop at 1 Hz.\n";
    while (!ioc.stopped()) 
    {
        std::cout << "[DEBUG] frequency: " << dji.get_frequency() << " Hz\n";

        // Drone name
        if (GET_ALL || GET_DRONE_NAME) {
            std::string drone_name = dji.get_drone_name();
            std::cout << "Drone name: " << drone_name <<"\n";
        }

        // Speed
        if (GET_ALL || GET_SPEED) {
            Speed3D speed = dji.get_speed();
            std::cout << "Speed: x=" << speed.x 
                      <<      ", y=" << speed.y 
                      <<      ", z=" << speed.z << "\n";
        }

        // Heading
        if (GET_ALL || GET_HEADING) {
            double heading = dji.get_heading();
            std::cout << "Heading: " << heading << "\n";
        }

        // Attitude
        if (GET_ALL || GET_ATTITUDE) {
            Attitude3D attitude = dji.get_attitude();
            std::cout << "Attitude: roll="  << attitude.roll
                      <<         ", pitch=" << attitude.pitch
                      <<         ", yaw="   << attitude.yaw << "\n";
        }

        // location
        if (GET_ALL || GET_LOCATION) {
            Location3D location = dji.get_location();
            std::cout << "Location: latitude="  << location.latitude
                      <<         ", longitude=" << location.longitude
                      <<         ", altitude="  << location.altitude << "\n";
        }

        // Gimbal Attitude
        if (GET_ALL || GET_GIMBAL_ATTITUDE) {
            Attitude3D attitude = dji.get_gimbal_attitude();
            std::cout << "Gimbal attitude: roll="  << attitude.roll
                      <<         ", pitch=" << attitude.pitch
                      <<         ", yaw="   << attitude.yaw << "\n";
        }

        // Gimbal Joint Attitude
        if (GET_ALL || GET_GIMBAL_JOINT_ATTITUDE) {
            Attitude3D attitude = dji.get_gimbal_joint_attitude();
            std::cout << "Gimbal joint attitude: roll="  << attitude.roll
                      <<         ", pitch=" << attitude.pitch
                      <<         ", yaw="   << attitude.yaw << "\n";
        }

        // Zoom focal length
        if (GET_ALL || GET_ZOOM_FOCAL_LENGTH) {
            double zoom_focal_length = dji.get_zoom_focal_length();
            std::cout << "Zoom focal length: " << zoom_focal_length << "\n";
        }

        // Hybrid focal length
        if (GET_ALL || GET_HYBRID_FOCAL_LENGTH) {
            double hybrid_focal_length = dji.get_hybrid_focal_length();
            std::cout << "Hybrid focal length: " << hybrid_focal_length << "\n";
        }

        // Optical focal length
        if (GET_ALL || GET_OPTICAL_FOCAL_LENGTH) {
            double optical_focal_length = dji.get_optical_focal_length();
            std::cout << "Optical focal length: " << optical_focal_length << "\n";
        }

        // Zoom ratio
        if (GET_ALL || GET_ZOOM_RATIO) {
            double zoom_ratio = dji.get_zoom_ratio();
            std::cout << "Zoom ratio: " << zoom_ratio << "\n";
        }

        // Battery level
        if (GET_ALL || GET_BATTERY_LEVEL) {
            int battery_level = dji.get_battery_level();
            std::cout << "Battery level: " << battery_level << "\n";
        }

        // Satellite count
        if (GET_ALL || GET_SATTELITE_COUNT) {
            double satellite_count = dji.get_satellite_count();
            std::cout << "Satellite count: " << satellite_count << "\n";
        }

        // Ready to takeoff
        if (GET_ALL || IS_READY_TO_TAKEOFF) {
            bool ready_to_takeoff = dji.is_ready_to_takeoff();
            std::cout << "Ready to takeoff: " << (ready_to_takeoff ? "true" : "false") << "\n";
        }

        // Takeoff block reason
        if (GET_ALL || GET_TAKEOFF_BLOCK_REASON) {
            std::string takeoff_block_reason = dji.get_takeoff_block_reason();
            std::cout << "Takeoff block reason: " << takeoff_block_reason << "\n";
        }

        // Home location
        if (GET_ALL || GET_HOME_LOCATION) {
            Location2D home_location = dji.get_home_location();
            std::cout << "Home location: latitude="  << home_location.latitude
                      <<              ", longitude=" << home_location.longitude << "\n";
        }

        // Distance to home
        if (GET_ALL || GET_DISTANCE_TO_HOME) {
            double distance_to_home = dji.get_distance_to_home();
            std::cout << "Distance to home: " << distance_to_home << "\n";
        }

        // LRF target
        if (GET_ALL || GET_LRF_TARGET) {
            Location3D lrf_target = dji.get_lrf_target();
            std::cout << "LRF target: latitude="  << lrf_target.latitude
                      <<           ", longitude=" << lrf_target.longitude
                      <<           ", altitude="  << lrf_target.altitude << "\n";
        }

        // Waypoint sequence
        if (GET_ALL || GET_WAYPOINT_SEQ) {
            int waypoint_seq = dji.get_waypoint_seq();
            std::cout << "Waypoint sequence: " << waypoint_seq << "\n";
        }

        // Waypoint reached
        if (GET_ALL || IS_WAYPOINT_REACHED) {
            bool waypoint_reached = dji.is_waypoint_reached(-1);
            std::cout << "Waypoint reached: " << (waypoint_reached ? "true" : "false") << "\n";
        }

        // Intermediary waypoint reached
        if (GET_ALL || IS_INTERMEDIARY_WAYPOINT_REACHED) {
            bool intermediary_waypoint_reached = dji.is_intermediary_waypoint_reached();
            std::cout << "Intermediary waypoint reached: " << (intermediary_waypoint_reached ? "true" : "false") << "\n";
        }

        // Yaw sequence
        if (GET_ALL || GET_YAW_SEQ) {
            int yaw_seq = dji.get_yaw_seq();
            std::cout << "Yaw sequence: " << yaw_seq << "\n";
        }

        // Yaw reached
        if (GET_ALL || IS_YAW_REACHED) {
            bool yaw_reached = dji.is_yaw_reached(-1);
            std::cout << "Yaw reached: " << (yaw_reached ? "true" : "false") << "\n";
        }

        // Altitude sequence
        if (GET_ALL || GET_ALTITUDE_SEQ) {
            int altitude_seq = dji.get_altitude_seq();
            std::cout << "Altitude sequence: " << altitude_seq << "\n";
        }

        // Altitude reached
        if (GET_ALL || IS_ALTITUDE_REACHED) {
            bool altitude_reached = dji.is_altitude_reached(-1);
            std::cout << "Altitude reached: " << (altitude_reached ? "true" : "false") << "\n";
        }

        // Camera recording
        if (GET_ALL || IS_CAMERA_RECORDING) {
            bool camera_recording = dji.is_camera_recording();
            std::cout << "Camera recording: " << (camera_recording ? "true" : "false") << "\n";
        }

        // Home set
        if (GET_ALL || IS_HOME_SET) {
            bool home_set = dji.is_home_set();
            std::cout << "Home set: " << (home_set ? "true" : "false") << "\n";
        }

        // Remaining flight time
        if (GET_ALL || GET_REMAINING_FLIGHT_TIME) {
            int remaining_flight_time = dji.get_remaining_flight_time();
            std::cout << "Remaining flight time: " << remaining_flight_time << "\n";
        }

        // Time needed to go home
        if (GET_ALL || GET_TIME_NEEDED_TO_GO_HOME) {
            double time_needed_to_go_home = dji.get_time_needed_to_go_home();
            std::cout << "Time needed to go home: " << time_needed_to_go_home << "\n";
        }

        // Time needed to land
        if (GET_ALL || GET_TIME_NEEDED_TO_LAND) {
            double time_needed_to_land = dji.get_time_needed_to_land();
            std::cout << "Time needed to land: " << time_needed_to_land << "\n";
        }

        // Total time
        if (GET_ALL || GET_TOTAL_TIME) {
            double total_time = dji.get_total_time();
            std::cout << "Total time: " << total_time << "\n";
        }

        // Max radius can fly and go home
        if (GET_ALL || GET_MAX_RADIUS_CAN_FLY_AND_GO_HOME) {
            double max_radius_can_fly_and_go_home = dji.get_max_radius_can_fly_and_go_home();
            std::cout << "Max radius can fly and go home: " << max_radius_can_fly_and_go_home << "\n";
        }

        // Remaining charge
        if (GET_ALL || GET_REMAINING_CHARGE) {
            int remaining_charge = dji.get_remaining_charge();
            std::cout << "Remaining charge: " << remaining_charge << "\n";
        }

        // Battery needed to land
        if (GET_ALL || GET_BATTERY_NEEDED_TO_LAND) {
            double battery_needed_to_land = dji.get_battery_needed_to_land();
            std::cout << "Battery needed to land: " << battery_needed_to_land << "\n";
        }

        // Battery needed to go home
        if (GET_ALL || GET_BATTERY_NEEDED_TO_GO_HOME) {
            double battery_needed_to_go_home = dji.get_battery_needed_to_go_home();
            std::cout << "Battery needed to go home: " << battery_needed_to_go_home << "\n";
        }

        // Serious low battery threshold
        if (GET_ALL || GET_SERIOUS_LOW_BATTERY_THRESHOLD) {
            double serious_low_battery_threshold = dji.get_serious_low_battery_threshold();
            std::cout << "Serious low battery threshold: " << serious_low_battery_threshold << "\n";
        }

        // Low battery threshold
        if (GET_ALL || GET_LOW_BATTERY_THRESHOLD) {
            double low_battery_threshold = dji.get_low_battery_threshold();
            std::cout << "Low battery threshold: " << low_battery_threshold << "\n";
        }

        // Flight mode
        if (GET_ALL || GET_FLIGHT_MODE) {
            std::string flight_mode = dji.get_flight_mode();
            std::cout << "Flight mode: " << flight_mode << "\n";
        }

        // Manual override active
        if (GET_ALL || IS_MANUAL_OVERRIDE_ACTIVE) {
            bool manual_override_active = dji.is_manual_override_active();
            std::cout << "Manual override active: " << (manual_override_active ? "true" : "false") << "\n";
        }
    }

    asio_thread.join(); // Wait for the Asio thread to finish

    std::cout << "[DEBUG] Program finished." << std::endl;
    return 0;
}