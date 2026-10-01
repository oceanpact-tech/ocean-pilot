/**
 ******************************************************************************
 * @file    http_handler.hpp
 * @brief   Interface of the asynchronous HTTP POST handler using coroutines.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


#pragma once


/* Includes ------------------------------------------------------------------*/

#include <string>
#include <functional>

// Asio
#include <boost/asio/awaitable.hpp>
#include <boost/asio/io_context.hpp>


/* Aliases  ------------------------------------------------------------------*/

namespace net = boost::asio;


/* Classes  ------------------------------------------------------------------*/

/**
 * @brief Asynchronous HTTP client for sending POST requests.
 * 
 * This class utilizes C++20 coroutines (net::awaitable) and the Boost.Beast 
 * library to execute non-blocking HTTP communications. It operates on a 
 * shared net::io_context to ensure efficient, concurrent network operations.
 */
class HttpHandler 
{
public:

    /**
     * @brief Constructor for the HttpHandler class.
     * 
     * @param ioc Reference to the application's central asynchronous event loop.
     * @param host Target IP address or hostname of the WildBridge server.
     * @param port Target port number (default is "8080").
     */
    HttpHandler(
        net::io_context& ioc, 
        std::string host, 
        std::string port)
        : ioc_(ioc), host_(std::move(host)), port_(std::move(port)) {}


    /**
     * @brief Sends a POST request asynchronously using coroutines.
     * 
     * @param endpoint The endpoint to which the request will be sent.
     * @param payload The payload to be sent.
     * @param on_complete A callback function to be called when the request is complete.
     */
    void post_async(
        const std::string endpoint, 
        const std::string payload,
        std::function<void()> on_complete = nullptr);

    /**
     * @brief Internal coroutine that executes the POST request asynchronously.
     * 
     * @param endpoint The endpoint to which the request will be sent.
     * @param payload The payload to be sent.
     * @param on_complete A callback function to be called when the request is complete.
     * @return net::awaitable<void> The coroutine awaitable object.
     */
    net::awaitable<void> do_post(
        std::string endpoint, 
        std::string payload, 
        std::function<void()> on_complete);
    

private:
    net::io_context& ioc_; /// Reference for the central asynchronous motor
    std::string host_;     /// RC IP address
    std::string port_;     /// Port number
};