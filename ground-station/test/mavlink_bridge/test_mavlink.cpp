/* Includes ------------------------------------------------------------------*/

#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/signal_set.hpp>

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action_server/action_server.hpp>
#include <mavsdk/plugins/telemetry_server/telemetry_server.hpp>
#include <mavsdk/plugins/param_server/param_server.hpp>

#include "dji_interface/dji_interface.hpp"

/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;

/* Entrypoint ----------------------------------------------------------------*/


int main(int argc, char *argv[])
{
    mavsdk::TelemetryServer::Position    *position = new mavsdk::TelemetryServer::Position();
    mavsdk::TelemetryServer::VelocityNed *velocity = new mavsdk::TelemetryServer::VelocityNed();
    mavsdk::TelemetryServer::Heading     *heading  = new mavsdk::TelemetryServer::Heading();

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

            delete position;
            delete velocity;
            delete heading;
            std::cout << "[System] Shutdown complete. Exiting.\n";
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
        std::cout << "oioi "<< dji.is_connected() << " && " << ioc.stopped() << "\n";

        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);

        std::cout << "[DEBUG - " << std::put_time(std::localtime(&current_time), "%H:%M:%S") << "] Waiting for telemetry stream to connect...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    mavsdk::Mavsdk::Configuration config(mavsdk::ComponentType::Autopilot);
    mavsdk::Mavsdk mavsdk{config};

    // --------------------------
    // CONNECTION SETUP
    // --------------------------
    mavsdk::ConnectionResult res = mavsdk.add_any_connection("udpout://127.0.0.1:14550");
    if (res != mavsdk::ConnectionResult::Success) {
        std::cerr << "Erro ao abrir conexão UDP: " << res << '\n';
        return 1;
    }
    std::cout << "[MAVLink] Conexão aberta. Heartbeats sendo enviados para o QGC...\n";

    auto server_component = mavsdk.server_component();
    mavsdk::TelemetryServer telemetry_server{server_component};

    std::cout << "[MAVLink] Enviando telemetria a 5Hz...\n";

    // Loop principal: O QGC espera um fluxo contínuo de dados
    while (!ioc.stopped()) {

        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        std::cout << "\n[" << std::put_time(std::localtime(&current_time), "%H:%M:%S") << "]"; 

        Location3D dji_location = dji.get_location();
        Speed3D    dji_speed    = dji.get_speed();
        double     dji_heading  = dji.get_heading();

        position->latitude_deg  = dji_location.latitude;
        position->longitude_deg = dji_location.longitude;
        position->absolute_altitude_m = dji_location.altitude; // Altitude do mar
        position->relative_altitude_m = dji_location.altitude; // Altitude de decolagem

        velocity->north_m_s = dji_speed.x;
        velocity->east_m_s  = dji_speed.y;
        velocity->down_m_s  = dji_speed.z;

        heading->heading_deg = dji_heading;

        telemetry_server.publish_position(*position, *velocity, *heading);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}