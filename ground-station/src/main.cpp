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
#include <boost/asio/io_context.hpp>


/* Aliases -------------------------------------------------------------------*/

namespace net = boost::asio;


/* Entrypoint ----------------------------------------------------------------*/

int main(int argc, char* argv[]) {

    // The io_context is required for all I/O
    net::io_context ioc;

    // Passa o maestro para a interface se conectar
    DJIInterface dji(ioc, "192.168.228.24");

    std::cout << "Conectado! Enviando comando de teste..." << std::endl;

    // Dispara a requisição e passa uma função lambda como callback
    dji.request_takeoff([]() {
        std::cout << "Callback executado internamente pela corrotina!" << std::endl;
    });

    // Run the I/O service. The call will return when POST operation is complete
    ioc.run();

    std::cout << "Fila de eventos vazia. Resposta processada e programa encerrando." << std::endl;

    return 0;
}