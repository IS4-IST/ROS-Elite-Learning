// JSON-RPC client for the Elite EC command interface, port 8055.
//
// Reference: EC SDK-Socket Script Manual Ver3.19.2, chapter 2.
//   send   {"jsonrpc":"2.0","method":<name>,"params":<params>,"id":<id>}\n
//   reply  {"jsonrpc":"2.0","result":"<JSON text>","id":<id>}
//          {"jsonrpc":"2.0","error":{"code":..,"message":..},"id":<id>}
// "result" is JSON text and can be encoded more than once ("\"V3.11.2\""),
// so it is unwrapped until it is no longer a JSON string.
//
// The controller accepts one client on 8055: use a single RpcClient per robot.
// Thread-safe: calls are serialised.
#pragma once

#include <asio.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "ec_robot/types.hpp"

namespace ec {

// The controller's own error replies ("not in remote mode", ...) as an error_code category.
// The code is the controller's numeric code; the message text is in Reply::value.
const std::error_category& controller_category();
 
struct Reply {
    std::error_code ec;      // empty = ok
    nlohmann::json value;    // decoded result, or the controller's error object
};
 
class RpcClient {
public:
    static constexpr std::uint16_t kPort = 8055;
 
    RpcClient(asio::io_context& context, std::string ip, std::uint16_t port = kPort,
              std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
    ~RpcClient();
    RpcClient(const RpcClient&) = delete;
    RpcClient& operator=(const RpcClient&) = delete;
 
    asio::awaitable<std::error_code> connect();   // opens the socket and starts the reader
    void close();                                 // fails every waiting call with not_connected
    bool connected() const { return connected_; }
 
    // Parameters by value: they live in the coroutine frame.
    asio::awaitable<Reply> call(std::string method, nlohmann::json params = nlohmann::json::array(),
                                std::optional<std::chrono::milliseconds> timeout = std::nullopt);
    void notify(std::string method, nlohmann::json params);   // no reply expected
 
private:
    struct Pending {
        explicit Pending(asio::io_context& context) : timer(context) {}
        asio::steady_timer timer;   // expires = timeout; cancelled by the reader = reply arrived
        bool done = false;
        Reply reply;
    };
 
    std::uint64_t send(const std::string& method, const nlohmann::json& params);
    asio::awaitable<void> writer();
    asio::awaitable<void> reader();
    void fail_all(std::error_code ec);
 
    asio::io_context& context_;
    std::string ip_;
    std::uint16_t port_;
    std::chrono::milliseconds timeout_;
    asio::ip::tcp::socket socket_;
    std::atomic<bool> connected_{false};
 
    std::uint64_t next_id_ = 0;
    std::deque<std::string> outbox_;   // deque: push_back keeps queued lines where they are
    bool writing_ = false;
    std::unordered_map<std::uint64_t, Pending*> pending_;
};

}  // namespace ec
