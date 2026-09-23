/**
 ******************************************************************************
 * @file    tcp_handler.cpp
 * @brief   Implementation of the raw TCP communication layer.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


/* Private Includes ----------------------------------------------------------*/

#include "transport/tcp_handler.hpp"

// Asio
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/detached.hpp>
#include <boost/asio/this_coro.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/as_tuple.hpp>


/* Public Class Methods ------------------------------------------------------*/

void TcpHandler::start_comm() 
{
    if (this->is_running_) 
        return;
    
    this->is_running_ = true;
    net::co_spawn(
        this->ioc_, 
        this->receiver_coroutine(), 
        net::detached
    );
}

void TcpHandler::stop_comm() 
{
    this->is_running_ = false;
}


/* Private Class Methods -----------------------------------------------------*/

net::awaitable<void> TcpHandler::receiver_coroutine() 
{
    try 
    {
        auto executor = co_await net::this_coro::executor;
        net::ip::tcp::resolver resolver(executor);
        net::ip::tcp::socket socket(executor);

        auto endpoints = co_await resolver.async_resolve(this->host_, this->port_, net::use_awaitable);
        co_await net::async_connect(socket, endpoints, net::use_awaitable);

        std::string buffer; 
        constexpr auto tolerant_token = net::as_tuple(net::use_awaitable);

        while (this->is_running_) 
        {
            auto [ec, n] = co_await net::async_read_until(
                socket,                        // Stream
                net::dynamic_buffer(buffer),   // Buffer
                "\n",                          // Delimiter
                tolerant_token                 // Token
            );
            
            // Error code treatment
            if (ec) 
            {
                std::cerr << "[TcpHandler] Connection interrupted: " << ec.message() << "\n";
                break;
            }

            std::string line = buffer.substr(0, n); // Extract line
            buffer.erase(0, n); // Clear dinamic buffer

            // Call the derived class's implementation to process the line
            this->process_raw_line(line);
        }
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "[TcpHandler] Erro de rede: " << e.what() << "\n";
    }
    
    this->is_running_ = false;
}