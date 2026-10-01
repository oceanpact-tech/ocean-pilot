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
#include <atomic>

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
        std::string port = "8081",
        bool frequency_debug = false,
        bool debug = false
    )
        : ioc_(ioc), 
          host_(std::move(host)),
          port_(std::move(port)),
          frequency_debug_(std::move(frequency_debug)),
          debug_(std::move(debug))
    {}

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

    /**
     * @brief Checks if the TCP communication is active.
     * @return true if the TCP communication is active, false otherwise.
     */
    bool is_connected() const { return is_connected_.load(); } /// Returns true if the TCP communication is active
    
    double frequency_hz_ = 0.0;              /// Frequency of the received lines in Hz

protected:

    /**
     * @brief Pure virtual function to be implemented by derived classes.
     * @param line A complete raw line extracted from the TCP stream (without '\n').
     */
    virtual void process_raw_line(const std::string& line) = 0;
    
    bool is_running_ = false; /// flag to indicate if the TCP communication is active
    
private:
    
    net::io_context& ioc_;                   /// Reference for the central asynchronous motor
    std::string host_;                       /// RC IP address
    std::string port_;                       /// Port number
    bool frequency_debug_ = false;           /// Flag to indicate if the frequency debug is active
    bool debug_ = false;                        /// Flag to indicate if the debug is active

    std::atomic<bool> is_connected_ = false; /// Flag to indicate if the TCP communication is active
    std::chrono::system_clock::time_point last_time_; /// Time point of the last received line
    
    /** 
     * @brief Coroutine that handles the TCP connection and reads lines from the socket.
     */
    net::awaitable<void> receiver_coroutine();
};