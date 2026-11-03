/**
 ******************************************************************************
 * @file    drone_discovery.cpp
 * @brief   Implementation of the UDP broadcast drone discovery.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#include "dji_interface/transport/drone_discovery.hpp"
#include <iostream>
#include <vector>

std::optional<std::string> DroneDiscovery::discover_drone(net::io_context& ioc, int timeout_seconds)
{
    try {
        udp::socket socket(ioc);
        socket.open(udp::v4());
        
        // Habilita a flag de broadcast no socket
        socket.set_option(boost::asio::socket_base::broadcast(true));

        // Envia o payload de descoberta para a porta 30000 no IP de broadcast universal
        udp::endpoint broadcast_endpoint(boost::asio::ip::address_v4::broadcast(), 30000);
        std::string payload = "DISCOVER_LYREBIRD";
        socket.send_to(boost::asio::buffer(payload), broadcast_endpoint);

        std::cout << "[DroneDiscovery] Broadcasting discovery message on port 30000...\n";

        // Prepara para receber a resposta
        std::vector<char> recv_buffer(1024);
        udp::endpoint sender_endpoint;
        
        // Aguarda a resposta com timeout bloqueante usando asio::steady_timer e run_one()
        // Uma abordagem mais limpa do que callbacks aninhados para fluxos de boot síncronos
        bool timeout = false;
        boost::asio::steady_timer timer(ioc, std::chrono::seconds(timeout_seconds));
        timer.async_wait([&](const boost::system::error_code& error) {
            if (!error) {
                timeout = true;
                socket.cancel(); // Cancela operações pendentes no socket (o receive)
            }
        });

        bool got_reply = false;
        std::string response_str;
        socket.async_receive_from(
            boost::asio::buffer(recv_buffer), sender_endpoint,
            [&](const boost::system::error_code& error, std::size_t bytes_recvd) {
                if (!error && !timeout) {
                    response_str.assign(recv_buffer.data(), bytes_recvd);
                    got_reply = true;
                    timer.cancel(); // Cancela o timer se a mensagem chegou primeiro
                }
            });

        // Roda o loop do io_context até que uma das operações (timer ou receive) termine
        ioc.restart();
        while (ioc.run_one()) {
            if (got_reply || timeout) break;
        }

        if (timeout) {
            std::cerr << "[DroneDiscovery] Timeout. Nenhum drone encontrado na rede local.\n";
            return std::nullopt;
        }

        if (got_reply) {
            // O protocolo responde no formato "LYREBIRD_HERE:<ip>:<name>"
            if (response_str.find("LYREBIRD_HERE:") == 0) {
                // Se o IP não foi preenchido no pacote, usamos o IP real do remetente (fallback)
                std::string drone_ip = sender_endpoint.address().to_string();
                std::cout << "[DroneDiscovery] Found Lyrebird drone at " << drone_ip << "\n";
                return drone_ip;
            }
        }
    } 
    catch (std::exception& e) {
        std::cerr << "[DroneDiscovery] Discovery error: " << e.what() << "\n";
    }

    return std::nullopt;
}