/**
 ******************************************************************************
 * @file    drone_discovery.cpp
 * @brief   Implementation of the UDP broadcast drone discovery.
 *
 * The discovery message is sent to the broadcast address of every active IPv4 
 * interface (and to the limited broadcast address) once per second until a 
 * Lyrebird reply arrives or the timeout expires. Replies are received on a 
 * fixed local port, so a firewall rule can allow them.
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */

#include "dji_interface/transport/drone_discovery.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <vector>


namespace
{
    constexpr unsigned short DISCOVERY_PORT = 30000;                 ///< Port the Lyrebird app listens on.
    constexpr unsigned short REPLY_PORT     = 30001;                 ///< Fixed local port for replies (allow it in the firewall).
    constexpr const char*    DISCOVERY_MSG  = "DISCOVER_WILDBRIDGE"; ///< Discovery request payload.
    constexpr const char*    REPLY_PREFIX   = "WILDBRIDGE_HERE:";    ///< Prefix of a valid reply.

    /// @brief Collects the broadcast address of every active, non-loopback IPv4 interface.
    std::vector<boost::asio::ip::address_v4> collect_broadcast_addresses()
    {
        std::vector<boost::asio::ip::address_v4> addresses;

        ifaddrs* interfaces = nullptr;
        if (getifaddrs(&interfaces) != 0)
            return addresses;

        for (ifaddrs* ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET)
                continue;

            const unsigned int flags = ifa->ifa_flags;
            if (!(flags & IFF_UP) || !(flags & IFF_BROADCAST) || (flags & IFF_LOOPBACK))
                continue;

            if (ifa->ifa_broadaddr == nullptr)
                continue;

            const auto* broadcast = reinterpret_cast<const sockaddr_in*>(ifa->ifa_broadaddr);
            addresses.emplace_back(ntohl(broadcast->sin_addr.s_addr));
        }

        freeifaddrs(interfaces);
        return addresses;
    }
}


std::optional<std::string> DroneDiscovery::discover_drone(net::io_context& ioc, int timeout_seconds)
{
    try 
    {
        udp::socket socket(ioc);
        socket.open(udp::v4());
        socket.set_option(boost::asio::socket_base::reuse_address(true));
        socket.set_option(boost::asio::socket_base::broadcast(true));

        boost::system::error_code bind_error;
        socket.bind(udp::endpoint(udp::v4(), REPLY_PORT), bind_error);
        if (bind_error)
            std::cerr << "[DroneDiscovery] Port " << REPLY_PORT << " unavailable, using ephemeral port.\n";

        // Destinations: broadcast of every interface + limited broadcast.
        std::vector<udp::endpoint> destinations;
        for (const auto& address : collect_broadcast_addresses())
            destinations.emplace_back(address, DISCOVERY_PORT);
        destinations.emplace_back(boost::asio::ip::address_v4::broadcast(), DISCOVERY_PORT);

        std::cout << "[DroneDiscovery] Broadcasting on port " << DISCOVERY_PORT << " to:";
        for (const auto& destination : destinations)
            std::cout << ' ' << destination.address().to_string();
        std::cout << '\n';

        // Receive loop: re-armed until a valid reply arrives.
        std::vector<char>          buffer(1024);
        udp::endpoint              sender;
        std::optional<std::string> drone_ip;

        std::function<void()> arm_receive = [&]() 
        {
            socket.async_receive_from(
                net::buffer(buffer), sender,
                [&](const boost::system::error_code& error, std::size_t bytes_received) 
                {
                    if (error)
                        return; // Closed or cancelled.

                    const std::string reply(buffer.data(), bytes_received);
                    if (reply.rfind(REPLY_PREFIX, 0) == 0) 
                    {
                        // The sender address is the one reachable from this machine.
                        drone_ip = sender.address().to_string();
                        return;
                    }

                    arm_receive(); // Someone else answered on this port: keep listening.
                });
        };
        arm_receive();

        const std::string payload(DISCOVERY_MSG);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds);

        while (!drone_ip && std::chrono::steady_clock::now() < deadline)
        {
            for (const auto& destination : destinations)
            {
                boost::system::error_code send_error;
                socket.send_to(net::buffer(payload), destination, 0, send_error);
            }

            // Wait up to 1 s for a reply, then send the request again.
            ioc.restart();
            ioc.run_one_for(std::chrono::seconds(1));
        }

        // Drain the pending receive handler while the locals it references are still alive.
        socket.close();
        ioc.restart();
        ioc.poll();

        if (!drone_ip)
        {
            std::cerr << "[DroneDiscovery] Timeout. No drone found. "
                         "(udp " << REPLY_PORT << ").\n";
            return std::nullopt;
        }

        std::cout << "[DroneDiscovery] Found WildBridge drone at " << *drone_ip << "\n";
        return drone_ip;
    } 
    catch (std::exception& e) 
    {
        std::cerr << "[DroneDiscovery] Discovery error: " << e.what() << "\n";
    }

    return std::nullopt;
}