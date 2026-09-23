/**
 ******************************************************************************
 * @file    tcp_handler.hpp
 * @brief   Abstract base class for asynchronous TCP line-based communication.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */

#pragma once


/* Includes ------------------------------------------------------------------*/

#include <string>
#include <iostream>

// Asio
#include <boost/asio/awaitable.hpp>
#include <boost/asio/io_context.hpp>


/* Aliases  ------------------------------------------------------------------*/

namespace net = boost::asio;


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Manages the raw TCP socket connection and network buffers.
 */
class TcpHandler 
{
public:

    /**
     * @brief Constructor for the TcpHandler class.
     * 
     * @param ioc Reference to the application's central asynchronous event loop.
     * @param host Target IP address or hostname of the WildBridge server.
     * @param port Target port number (default is "8081").
     */
    TcpHandler(
        net::io_context& ioc, 
        std::string host, 
        std::string port = "8081")
        : ioc_(ioc), host_(std::move(host)), port_(std::move(port)) {}

    /**
     * @brief Destructor for the TCPHandler
     * 
     * Stops the TCP communication if it is still running.
     */
    virtual ~TcpHandler() { this->stop_comm(); }

    /**
     * @brief Starts the asynchronous TCP receiving coroutine.
     */
    void start_comm();

    /**
     * @brief Stops the asynchronous TCP communication and terminates the receiving coroutine.
     */
    void stop_comm();

protected:

    /**
     * @brief Pure virtual function to be implemented by derived classes.
     * @param line A complete raw line extracted from the TCP stream (without '\n').
     */
    virtual void process_raw_line(const std::string& line) = 0;

    bool is_running_ = false; /// flag to indicate if the TCP communication is active

private:

    net::io_context& ioc_; /// Reference for the central asynchronous motor
    std::string host_;     /// RC IP address
    std::string port_;     /// Port number

    /** 
     * @brief Coroutine that handles the TCP connection and reads lines from the socket.
     */
    net::awaitable<void> receiver_coroutine();
};