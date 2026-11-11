// Small networking helpers shared by RpcClient and Monitor.
#pragma once

#include <utility>   // before asio.hpp: Asio 1.18.1 uses std::exchange without including it
#include "asio.hpp"

#include <chrono>
#include <string>

namespace ec {

// Connect `socket` to ip:port, giving up after `timeout`. No exceptions: refused, unreachable,
// timed_out (the timer won) or invalid_argument (not an IP address) come back as the error code.
inline asio::awaitable<std::error_code> connect_with_timeout(asio::ip::tcp::socket& socket, const std::string& ip,
                                                             unsigned short port, std::chrono::milliseconds timeout) {
    std::error_code ec;
    const asio::ip::tcp::endpoint endpoint(asio::ip::make_address(ip, ec), port);
    if (ec) co_return ec;

    // If the timer fires first it closes the socket, which ends the connect with operation_aborted.
    // (Asio 1.18 has no awaitable operators for racing two operations.)
    asio::steady_timer timer(socket.get_executor());
    timer.expires_after(timeout);
    timer.async_wait([&socket](std::error_code e) {
        if (!e) socket.close();
    });
    co_await socket.async_connect(endpoint, asio::redirect_error(asio::use_awaitable, ec));
    const bool timed_out = timer.expiry() <= asio::steady_timer::clock_type::now();
    timer.cancel();
    if (ec) {
        std::error_code ignored;
        socket.close(ignored);
        co_return (timed_out && ec == asio::error::operation_aborted) ? std::error_code(asio::error::timed_out) : ec;
    }
    socket.set_option(asio::ip::tcp::no_delay(true), ec);   // no Nagle delay on small messages
    co_return ec;
}

}  // namespace ec
