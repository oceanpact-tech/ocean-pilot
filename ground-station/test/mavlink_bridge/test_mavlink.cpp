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


/* Defines -------------------------------------------------------------------*/

#define SIMULATION_MODE 1 // 1 = Simulation mode (localhost), 0 = Real drone (


/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;


/* Entrypoint ----------------------------------------------------------------*/


int main(int argc, char *argv[])
{
    // buffers
    mavsdk::TelemetryServer::Position    *position = new mavsdk::TelemetryServer::Position();
    mavsdk::TelemetryServer::VelocityNed *velocity = new mavsdk::TelemetryServer::VelocityNed();
    mavsdk::TelemetryServer::Heading     *heading  = new mavsdk::TelemetryServer::Heading();
    mavsdk::TelemetryServer::EulerAngle  *attitude = new mavsdk::TelemetryServer::EulerAngle();
    mavsdk::TelemetryServer::AngularVelocityBody *angular_velocity = new mavsdk::TelemetryServer::AngularVelocityBody();

    // The io_context is required for all I/O
    net::io_context ioc;
    auto work_guard = net::make_work_guard(ioc);

    // Connect to the drone using the DJIInterface
    std::string host = SIMULATION_MODE ? "0.0.0.0" : "192.168.228.24";
    DJIInterface dji(ioc, host, true, true);

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
            delete attitude;
            delete angular_velocity;
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
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        std::cout << "[DEBUG - " << std::put_time(std::localtime(&current_time), "%H:%M:%S") << "] Waiting for telemetry stream to connect...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // --------------------------
    // CONNECTION SETUP
    // --------------------------

    const std::pair<const char*, int> int_params[] = {
        {"SYS_AUTOSTART", 4001}, 
        {"SYS_AUTOCONFIG", 0}, 
        {"MAV_SYS_ID", 1},
        {"COM_RC_IN_MODE", 0},
        {"RC_MAP_ROLL", 0}, 
        {"RC_MAP_PITCH", 0}, 
        {"RC_MAP_YAW", 0}, 
        {"RC_MAP_THROTTLE", 0},
        {"RC_MAP_FLAPS", 0}, 
        {"RC_MAP_AUX1", 0}, 
        {"RC_MAP_AUX2", 0},
        {"COM_FLTMODE1", -1}, 
        {"COM_FLTMODE2", -1}, 
        {"COM_FLTMODE3", -1},
        {"COM_FLTMODE4", -1}, 
        {"COM_FLTMODE5", -1}, 
        {"COM_FLTMODE6", -1},
        {"NAV_DLL_ACT", 0}, 
        {"NAV_RCL_ACT", 0}, 
        {"COM_LOW_BAT_ACT", 0},
        {"CAL_GYRO0_ID", 1}, 
        {"CAL_ACC0_ID", 1},
        {"CAL_MAG0_ID", 1}, 
        {"CAL_MAG1_ID", 1}, 
        {"CAL_MAG2_ID", 1},
    };

    mavsdk::Mavsdk::Configuration config(mavsdk::ComponentType::Autopilot);
    config.set_mav_type(mavsdk::MavType::Quadrotor);
    config.set_autopilot(mavsdk::Autopilot::Px4);
    mavsdk::Mavsdk mavsdk{config};

    // TELEMETRY SERVER 

    auto server_component = mavsdk.server_component();
    mavsdk::TelemetryServer telemetry_server{server_component};

    // ACTION SERVER

    mavsdk::ActionServer action_server{server_component};

    mavsdk::ActionServer::AllowableFlightModes allowed_modes{};
    allowed_modes.can_auto_mode = true;      // Habilita modos autônomos (Mission, RTL)
    allowed_modes.can_guided_mode = true;    // Habilita modos de clique no mapa (Hold, Goto)
    allowed_modes.can_stabilize_mode = true; // Habilita modos manuais
    action_server.set_allowable_flight_modes(allowed_modes);
    action_server.set_armable(true, true);     // (armable, force_armable)
    action_server.set_disarmable(true, true);
    action_server.set_flight_mode(mavsdk::ActionServer::FlightMode::Hold); // Set current flight mode

    // PARAM SERVER SETUP 

    mavsdk::ParamServer param_server{server_component};

    param_server.provide_param_int("SYS_AUTOSTART", 4001); // Exemplo de parâmetro

    for (const auto& [name, value] : int_params) {
        param_server.provide_param_int(name, value);
    }

    const std::pair<const char*, float> float_params[] = {
        {"RTL_LAND_DELAY", 0.0f}, {"RTL_DESCEND_ALT", 30.0f},
        {"RTL_RETURN_ALT", 60.0f}, {"COM_RC_LOSS_T", 0.5f},
    };
    for (const auto& [name, value] : float_params) {
        param_server.provide_param_float(name, value);
    }

    param_server.subscribe_changed_param_int(
        [](mavsdk::ParamServer::IntParam param) {
            std::cout << "[ParamServer] QGC pediu para alterar (Int): " << param.name << " para " << param.value << '\n';
        }
    );
    param_server.subscribe_changed_param_float(
        [](mavsdk::ParamServer::FloatParam param) {
            std::cout << "[ParamServer] QGC pediu para alterar (Float): " << param.name << " para " << param.value << '\n';
        }
    );

    // CONNECTION TO QGC

    mavsdk::ConnectionResult res = mavsdk.add_any_connection("udpout://127.0.0.1:14550");
    if (res != mavsdk::ConnectionResult::Success) {
        std::cerr << "Erro ao abrir conexão UDP: " << res << '\n';
        return 1;
    }
    std::cout << "[MAVLink] Conexão aberta. Heartbeats sendo enviados para o QGC...\n";


    std::cout << "[MAVLink] Enviando telemetria a 5Hz...\n";

    // Loop principal: O QGC espera um fluxo contínuo de dados
    while (!ioc.stopped()) {

        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);
        //std::cout << "\n[" << std::put_time(std::localtime(&current_time), "%H:%M:%S") << "] STATUS: RUNNING"; 

        Location3D dji_location = dji.get_location();
        Speed3D    dji_speed    = dji.get_speed();
        double     dji_heading  = dji.get_heading();
        Attitude3D dji_attitude = dji.get_attitude();

        position->latitude_deg  = dji_location.latitude;
        position->longitude_deg = dji_location.longitude;
        position->absolute_altitude_m = dji_location.altitude; // Altitude do mar
        position->relative_altitude_m = dji_location.altitude; // Altitude de decolagem

        velocity->north_m_s = dji_speed.x;
        velocity->east_m_s  = dji_speed.y;
        velocity->down_m_s  = dji_speed.z;

        heading->heading_deg = dji_heading;

        telemetry_server.publish_position(*position, *velocity, *heading);

        attitude->roll_deg  = dji_attitude.roll;
        attitude->pitch_deg = dji_attitude.pitch;
        attitude->yaw_deg   = dji_attitude.yaw;

        telemetry_server.publish_attitude(*attitude, *angular_velocity);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    asio_thread.join();
    return 0;
}