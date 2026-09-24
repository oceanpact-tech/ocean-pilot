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

#include "dji_interface.hpp"

#include <iostream>
#include <thread>
#include <chrono>

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/signal_set.hpp>


/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;


/* Definitions ---------------------------------------------------------------*/
#define GET_ALL                        0
#define GET_DRONE_NAME                 0
#define GET_SPEED                      0
#define GET_HEADING                    0
#define GET_ATTITUDE                   0
#define GET_LOCATION                   0
#define GET_GIMBAL_ATTITUDE            0
#define GET_GIMBAL_JOINT_ATTITUDE      0
#define GET_ZOOM_FOCAL_LENGTH          0
#define GET_HYBRID_FOCAL_LENGTH        0
#define GET_OPTICAL_FOCAL_LENGTH       0
#define GET_ZOOM_RATIO                 0
#define GET_BATTERY_LEVEL              1
#define GET_SATTELITE_COUNT            1


/* Entrypoint ----------------------------------------------------------------*/

int main(int argc, char* argv[]) 
{

    // The io_context is required for all I/O
    net::io_context ioc;
    auto work_guard = net::make_work_guard(ioc);

    // Connect to the drone using the DJIInterface
    DJIInterface dji(ioc, "127.0.0.1");
    std::cout << "[DEBUG] Drone connected.\n";

    // Set up signal handling for graceful shutdown
    net::signal_set signals(ioc, SIGINT, SIGTERM);

    // Book closing coroutine
    signals.async_wait([&](const boost::system::error_code& error, int signal_number) {
        if (!error) 
        {
            std::cout << "\n[System] Interruption detected (Signal " << signal_number << ").\n";
            std::cout << "[Sistema] Starting graceful shutdown sequence...\n";
            
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

    // Control loop
    std::cout << "[DEBUG] Starting control loop at 1 Hz.\n";
    while (!ioc.stopped()) 
    {
        std::cout << "\n";

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

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    asio_thread.join(); // Wait for the Asio thread to finish

    std::cout << "[DEBUG] Program finished." << std::endl;
    return 0;
}