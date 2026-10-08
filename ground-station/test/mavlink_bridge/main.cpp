/* Includes ------------------------------------------------------------------*/

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/signal_set.hpp>

#include <boost/json.hpp>

#include "mavlink_bridge/mavlink_bridge.hpp"


/* Defines -------------------------------------------------------------------*/

#define SIMULATION_MODE 1 // 1 = Simulation mode (localhost), 0 = Real drone (


/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;


/* Entrypoint ----------------------------------------------------------------*/


int main(int argc, char *argv[])
{
    // The io_context is required for all I/O
    net::io_context ioc;
    auto work_guard = net::make_work_guard(ioc);

    // Connect to the drone using the DJIInterface
    std::string drone_ip = SIMULATION_MODE ? "0.0.0.0" : "192.168.228.24";

    WildBridgeSocketConfig wildbridge_socket = {
        .ip = drone_ip,
        .command_port = "8080",
        .telemetry_port = "8081"
    };

    GroundStationSocketConfig ground_station_socket = {
        .ip = "127.0.0.1", // QGC Running locally
        .port = "14550"
    };

    // MavlinkBridge objetc: connect the drone with ground station
    MavlinkBridge bridge(ioc, wildbridge_socket, ground_station_socket);
    if (!bridge.start())
    {
        std::cerr << "[Main] Fail to start MavlinkBridge.\n";
        return -1;
    }

    // Set up signal handling for graceful shutdown
    net::signal_set signals(ioc, SIGINT, SIGTERM);

    // Book closing coroutine
    signals.async_wait([&](const boost::system::error_code& error, int signal_number) {
        if (!error) 
        {
            std::cout << "\n[Main] Interruption detected (Signal " << signal_number << ").\n";
            std::cout << "[Main] Starting graceful shutdown sequence...\n";
            
            // Stop telemetry stream and clean up
            bridge.stop();
            work_guard.reset(); 
            ioc.stop();

            std::cout << "[Main] Shutdown complete. Exiting.\n";
        }
    });

    // Run the IO service in background.
    std::cout << "[Main] IO Context initialized. Press Ctrl+C to stop the program.\n";
    ioc.run();

    return 0;
}