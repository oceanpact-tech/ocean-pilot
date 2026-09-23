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

#include "dji_interface.hpp"

#include <iostream>
#include <thread>
#include <chrono>

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>


/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;


/* Entrypoint ----------------------------------------------------------------*/

int main(int argc, char* argv[]) {

    // The io_context is required for all I/O
    net::io_context ioc;
    auto work_guard = net::make_work_guard(ioc);

    // Passa o maestro para a interface se conectar
    DJIInterface dji(ioc, "192.168.228.24");

    std::cout << "Conectado! Enviando comando de teste..." << std::endl;

    // Dispara a requisição e passa uma função lambda como callback
    // dji.request_takeoff([]() {
    //     std::cout << "Callback executado internamente pela corrotina!" << std::endl;
    // });

    // Run the IO service in background.
    std::thread asio_thread([&ioc]() {
        ioc.run();
    });

    std::cout << "[Main] Sistema iniciado. Conectando telemetria..." << std::endl;

    dji.start_telemetry_stream();

    // Espera 1 segundo para o socket TCP estabilizar e receber os primeiros JSONs
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. Exemplo de co_spawn em tempo de execução!
    // Simulando um evento que acabou de chegar do piloto:
    // std::cout << "[Main] Comando do piloto recebido. Solicitando Takeoff..." << std::endl;
    // dji.request_takeoff([]() {
    //     // Isso aqui vai executar na thread do Asio quando o HTTP POST terminar
    //     std::cout << "[Rede] Callback de Takeoff confirmado pelo drone!" << std::endl;
    // });

    // 4. O seu Loop de Aplicação (Fila de eventos, controle de voo, MAVLink, etc)
    std::cout << "[Main] Iniciando loop de controle (1 Hz)..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        
        // Acessa a telemetria completa processada pela thread do Asio
        Vector3D telemetry = dji.get_speed();
        
        // ESTE BLOCO NAO É MAIS VALIDO
        // if (!telemetry.empty()) {
        //     // O boost::json::serialize converte o objeto JSON de volta para uma string formatada
        //     std::cout << "[Voo] Telemetria completa: " 
        //               << boost::json::serialize(telemetry) << std::endl;
        // } else {
        //     std::cout << "[Voo] Aguardando dados de telemetria..." << std::endl;
        // }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 5. Encerramento limpo
    std::cout << "[Main] Desligando subsistemas..." << std::endl;
    dji.stop_telemetry_stream();

    work_guard.reset(); // Libera o maestro para encerrar quando a fila esvaziar
    ioc.stop();         // Força a parada de operações pendentes
    asio_thread.join(); // Aguarda a thread de rede fechar com segurança

    std::cout << "[Main] Programa encerrado." << std::endl;
    return 0;
}