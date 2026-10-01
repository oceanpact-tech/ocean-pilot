/**
 ******************************************************************************
 * @file    main.cpp
 * @brief   Main entry point for the MAVLink-to-DJI WildBridge application.
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

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/signal_set.hpp>


/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;


/* Entrypoint ----------------------------------------------------------------*/

int main(int argc, char* argv[]) {

    // The io_context is required for all I/O
    net::io_context ioc;
    auto work_guard = net::make_work_guard(ioc);

    // Connect to the drone using the DJIInterface
    DJIInterface dji(ioc, "192.168.228.24");

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

    asio_thread.join(); // Wait for the Asio thread to finish

    std::cout << "[DEBUG] Program finished." << std::endl;
    return 0;
}