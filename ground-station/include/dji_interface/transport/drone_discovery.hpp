/**
 ******************************************************************************
 * @file    drone_discovery.hpp
 * @brief   UDP broadcast discovery for locating the Lyrebird Android app.
 *
 * This file defines the DroneDiscovery class, which implements a UDP 
 * broadcast mechanism to dynamically discover the drone's IP address 
 * on the local network, eliminating the need for hardcoded IPs.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#pragma once

#include <string>
#include <optional>
#include <boost/asio.hpp>

namespace net = boost::asio;
using udp = boost::asio::ip::udp;

class DroneDiscovery 
{
public:
    /**
     * @brief Performs a blocking UDP broadcast to discover the drone.
     * 
     * @param ioc     The ASIO context to use for the socket.
     * @param timeout The maximum time (in seconds) to wait for a reply.
     * @return std::optional<std::string> The discovered IP address, or nullopt if it fails.
     */
    static std::optional<std::string> discover_drone(net::io_context& ioc, int timeout_seconds = 5);
};