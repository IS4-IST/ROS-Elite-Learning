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

class RpcClient {
public:
    static constexpr std::uint16_t kPort = 8055;

    explicit RpcClient(std::string ip, std::uint16_t port = kPort,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
    ~RpcClient();
    RpcClient(const RpcClient&) = delete;
    RpcClient& operator=(const RpcClient&) = delete;

    void connect();
    void close();
    bool connected() const;

    // Send a command and wait for its reply; returns the decoded result.
    // Throws ECError on an error reply, timeout or lost connection.
    nlohmann::json call(const std::string& method,
                        const nlohmann::json& params = nlohmann::json::array(),
                        std::optional<std::chrono::milliseconds> timeout = std::nullopt);

    // Send without waiting for a reply. Only for commands configured not to reply,
    // e.g. tt_put_servo_joint_to_buf after transparent_transmission_init(response_enable=0).
    void notify(const std::string& method, const nlohmann::json& params);

private:
    std::uint64_t send_locked(const std::string& method, const nlohmann::json& params);
    std::string read_line_locked(Clock::time_point deadline, const std::string& method);

    std::string ip_;
    std::uint16_t port_;
    std::chrono::milliseconds timeout_;
    asio::io_context io_;
    asio::ip::tcp::socket socket_;
    asio::streambuf buffer_;
    mutable std::mutex mutex_;
    std::uint64_t next_id_ = 0;
    bool connected_ = false;
};

// Unwrap a JSON-text result (exposed for tests)
nlohmann::json decode_result(const nlohmann::json& result);

}  // namespace ec
