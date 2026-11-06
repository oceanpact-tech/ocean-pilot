/**
 ******************************************************************************
 * @file    http_handler.cpp
 * @brief   Implementation of the HttpHandler class.
 *
 * @author  Thiago Marques de Oliveira
 * @date    September, 2026
 ******************************************************************************
 */


/* Private Includes ----------------------------------------------------------*/

#include "dji_interface/transport/http_handler.hpp"

#include <iostream>

// Asio
#include <boost/asio/co_spawn.hpp>
#include <boost/asio/as_tuple.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/this_coro.hpp>
#include <boost/asio/detached.hpp>

// Beast
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>


/* Private Aliases -----------------------------------------------------------*/

namespace beast = boost::beast;
namespace http = beast::http;

using tcp = net::ip::tcp;


/* Public Class Methods ------------------------------------------------------*/

void HttpHandler::post_async(
    std::string endpoint, 
    std::string payload,
    std::function<void(bool)> on_complete) 
{
    // Launch the asynchronous operation
    net::co_spawn(
        this->ioc_, 
        do_post(
            std::move(endpoint), 
            std::move(payload), 
            [callback = std::move(on_complete)](bool success, std::string /*ignored_response*/) {
                if (callback)
                    callback(success);
            }
        ), 
        net::detached
    );
}

void HttpHandler::post_async(
    std::string endpoint, 
    std::string payload,
    std::function<void(bool, std::string)> on_complete) 
{
    // Launch the asynchronous operation
    net::co_spawn(
        this->ioc_, 
        do_post(
            std::move(endpoint), 
            std::move(payload), 
            on_complete
        ), 
        net::detached
    );
}


/* Private Class Methods -----------------------------------------------------*/

net::awaitable<void> HttpHandler::do_post(
    std::string endpoint, 
    std::string payload,
    std::function<void(bool, std::string)> on_complete)
{
    try {
        auto executor = co_await net::this_coro::executor;
        auto resolver = tcp::resolver{ executor };
        auto stream   = beast::tcp_stream{ executor };

        // Look up the domain name
        auto const results = co_await resolver.async_resolve(this->host_, this->port_);
        
        // Set the timeout
        stream.expires_after(std::chrono::seconds(10));

        // Make the connection on the IP address we get from a lookup
        co_await stream.async_connect(results);

        // Set up an HTTP POST request message
        http::request<http::string_body> req{ http::verb::post, endpoint, 11 };
        req.set(http::field::host, this->host_);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
        req.set(http::field::content_type, "text/plain");
        req.set(http::field::connection, "close");
        req.body() = payload;
        req.prepare_payload();

        // Set the timeout
        stream.expires_after(std::chrono::seconds(10));

        // Send the HTTP request to the remote host
        co_await http::async_write(stream, req);

        // This buffer is used for reading and must be persisted
        beast::flat_buffer buffer;

        // Declare a container to hold the response
        http::response<http::string_body> res;

        constexpr auto tolerant_awaitable = net::as_tuple(net::use_awaitable);
        auto [ec, bytes_read] = co_await http::async_read(stream, buffer, res, tolerant_awaitable);

        beast::error_code shutdown_ec;
        stream.socket().shutdown(tcp::socket::shutdown_both, shutdown_ec);

        bool success = (!ec || ec == http::error::end_of_stream) && (res.result_int() == 200);
        
        std::string response_body = res.body();

        if (on_complete) on_complete(success, response_body);
    } 
    catch (std::exception const& e) {
        std::cerr << "[HttpHandler] Exception on POST " << endpoint << ": " << e.what() << "\n";
        if (on_complete) on_complete(false, "-1");
    }
}