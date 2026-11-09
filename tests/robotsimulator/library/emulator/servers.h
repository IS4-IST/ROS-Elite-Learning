// TCP servers that make the emulator look like the real controller on the network:
//   port 8055  JSON-RPC, one client at a time (like the real controller)
//   port 8056  monitor stream, one 1024-byte packet per control tick to every client
// Both run on the io_context of the control thread.
#pragma once

#include <asio.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "emulator/controller.h"

namespace emu {

class RpcServer {
public:
    RpcServer(asio::io_context& io, Controller& controller, const std::string& bind, unsigned short port = 8055);
    bool connected() const;   // a client holds the 8055 connection

private:
    struct Session;
    void accept();

    asio::io_context& io_;
    Controller& controller_;
    asio::ip::tcp::acceptor acceptor_;
    std::weak_ptr<Session> active_;
};

class MonitorServer {
public:
    MonitorServer(asio::io_context& io, const std::string& bind, unsigned short port = 8056);
    void broadcast(std::vector<std::uint8_t> packet);   // call once per control tick
    std::size_t clients() const { return clients_.size(); }

private:
    struct Client {
        asio::ip::tcp::socket socket;
        bool writing = false;
        explicit Client(asio::ip::tcp::socket s) : socket(std::move(s)) {}
    };
    void accept();

    asio::ip::tcp::acceptor acceptor_;
    std::vector<std::shared_ptr<Client>> clients_;
};

}  // namespace emu
