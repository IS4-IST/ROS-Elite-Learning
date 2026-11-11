#include "ec_robot/rpc_client.h"
#include <cstdio>
using namespace std::chrono_literals;

asio::awaitable<void> session(asio::io_context& io, ec::RpcClient& rpc) {
    if (auto ec = co_await rpc.connect()) { std::printf("connect: %s\n", ec.message().c_str()); co_return; }
    std::printf("connected\n");

    auto v = co_await rpc.call("getSoftVersion");
    std::printf("1. version: %s %s\n", v.ec ? v.ec.message().c_str() : "ok", v.value.dump().c_str());

    // 2. two calls in flight at once, each gets its own reply
    int finished = 0;
    for (const char* m : {"getRobotMode", "getRobotState"})
        asio::co_spawn(io, [&rpc, &finished, m]() -> asio::awaitable<void> {
            auto r = co_await rpc.call(m);
            std::printf("2. %s -> %s %s\n", m, r.ec ? r.ec.message().c_str() : "ok", r.value.dump().c_str());
            ++finished;
        }, asio::detached);
    asio::steady_timer t(io);
    while (finished < 2) { t.expires_after(5ms); co_await t.async_wait(asio::use_awaitable); }

    // 3. a controller error reply comes back as an error_code, no exception
    auto bad = co_await rpc.call("noSuchMethod");
    std::printf("3. noSuchMethod -> %s (%s) %s\n", bad.ec.message().c_str(), bad.ec.category().name(), bad.value.dump().c_str());

    // 4. a timeout far shorter than the round trip; the late reply is then dropped
    auto slow = co_await rpc.call("getSoftVersion", nlohmann::json::array(), 0ms);
    std::printf("4. 0 ms timeout -> %s\n", slow.ec.message().c_str());
    auto after = co_await rpc.call("getRobotMode");
    std::printf("4. next call still gets its own reply -> %s %s\n", after.ec ? after.ec.message().c_str() : "ok", after.value.dump().c_str());

    // 5. notify: no reply expected, nothing waits
    rpc.notify("getRobotMode", nlohmann::json::array());
    std::printf("5. notify sent\n");

    // 6. close while a call waits: it is woken with not_connected
    asio::co_spawn(io, [&rpc]() -> asio::awaitable<void> {
        auto r = co_await rpc.call("getSoftVersion", nlohmann::json::array(), 5000ms);
        std::printf("6. call during close -> %s\n", r.ec.message().c_str());
    }, asio::detached);
    rpc.close();
    t.expires_after(100ms); co_await t.async_wait(asio::use_awaitable);
    auto closed = co_await rpc.call("getRobotMode");
    std::printf("6. call after close -> %s\n", closed.ec.message().c_str());
}

int main() {
    asio::io_context io;
    ec::RpcClient rpc(io, "127.0.0.1");
    asio::co_spawn(io, session(io, rpc), asio::detached);
    io.run();
}
