#pragma once

#include <string>
#include <thread>
#include <iostream>
#include <functional>

#include <boost/asio.hpp>
#include <boost/beast.hpp>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;

using tcp = net::ip::tcp;

class HttpHandler 
{
public:

    /* @brief Class constructor
     * @param host 
     * @param port
     */
    HttpHandler(std::string host, std::string port = "8080")
        : host_(std::move(host)), port_(std::move(port)) {}


    // TO DO: corrigir comentario
    /* @brief Stops telemetry stream
     * @param endpoint The endpoint to which the command will be sent
     * @param payload The payload to be sent
     * @return none
     */
    void post_async(
        const std::string& endpoint, 
        const std::string& payload,
        std::function<void()> on_complete = nullptr);

private:
    std::string host_;
    std::string port_;
};