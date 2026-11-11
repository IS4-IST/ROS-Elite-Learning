#include "ec_robot/monitor.h"
#include "ec_robot/rpc_client.h"
#include <cmath>
#include <cstdio>
#include <thread>
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

asio::awaitable<void> tests(asio::io_context& io) {
    asio::steady_timer t(io);

    // 1. connect, first packet, latest()
    ec::Monitor mon(io, "127.0.0.1");
    if (auto ec = co_await mon.connect()) { std::printf("1. connect: %s\n", ec.message().c_str()); co_return; }
    const auto s = mon.latest();
    std::printf("1. connected; latest q = [%.1f %.1f %.1f %.1f %.1f %.1f], mode %d, servo %d\n",
                s->q()[0], s->q()[1], s->q()[2], s->q()[3], s->q()[4], s->q()[5], int(s->robot_mode), s->servo_ready);

    // 2. packet rate and gaps, waiting packet by packet
    double max_gap = 0, sum = 0;
    auto prev = Clock::now();
    for (int i = 0; i < 125; ++i) {
        const auto res = co_await mon.wait_until([](const ec::RobotStatus&) { return true; }, 200ms);
        const ec::RobotStatus& st = res.second;
        const double gap = std::chrono::duration<double, std::milli>(st.received - prev).count();
        if (i > 0) { max_gap = std::max(max_gap, gap); sum += gap; }
        prev = st.received;
    }
    std::printf("2. 125 packets: mean gap %.2f ms, max %.2f ms, invalid %llu\n", sum / 124, max_gap,
                (unsigned long long)mon.invalid_packets());

    // 3. a condition that never holds -> timed_out, no exception
    const auto never = co_await mon.wait_until([](const ec::RobotStatus& st) { return st.estop; }, 300ms);
    std::printf("3. wait for e-stop (never) -> %s\n", never.first.message().c_str());

    // 4. both clients on one io_context: switch the servos on over RPC, wait for the monitor to see it
    ec::RpcClient rpc(io, "127.0.0.1");
    if (auto ec = co_await rpc.connect()) { std::printf("4. rpc connect: %s\n", ec.message().c_str()); co_return; }
    co_await rpc.call("clearAlarm");
    co_await rpc.call("syncMotorStatus");
    const auto t0 = Clock::now();
    // build arguments first: a braced list inside a co_await expression crashes GCC (11-13)
    nlohmann::json servo_on = {{"status", 1}};
    auto r = co_await rpc.call("set_servo_status", servo_on);
    const auto on = co_await mon.wait_until([](const ec::RobotStatus& st) { return st.servo_ready; }, 3000ms);
    std::printf("4. set_servo_status -> %s; monitor saw servo on after %.0f ms -> %s\n",
                r.ec ? r.ec.message().c_str() : "ok",
                std::chrono::duration<double, std::milli>(Clock::now() - t0).count(), on.first ? on.first.message().c_str() : "ok");

    // 5. latest() from another thread while the io_context keeps running
    std::thread other([&mon] {
        int seen = 0;
        for (int i = 0; i < 50; ++i) { if (auto s = mon.latest()) seen += s->servo_ready; std::this_thread::sleep_for(2ms); }
        std::printf("5. other thread read latest() 50 times, servo on in %d\n", seen);
    });
    t.expires_after(200ms); co_await t.async_wait(asio::use_awaitable);
    other.join();

    // 6. close while someone waits -> woken with not_connected; later waits too
    asio::co_spawn(io, [&mon]() -> asio::awaitable<void> {
        const auto res = co_await mon.wait_until([](const ec::RobotStatus& st) { return st.estop; }, 5000ms);
        std::printf("6. waiting during close -> %s\n", res.first.message().c_str());
    }, asio::detached);
    t.expires_after(50ms); co_await t.async_wait(asio::use_awaitable);
    mon.close();
    t.expires_after(50ms); co_await t.async_wait(asio::use_awaitable);
    const auto after = co_await mon.wait_until([](const ec::RobotStatus&) { return true; }, 100ms);
    std::printf("6. wait after close -> %s; last_error: %s\n", after.first.message().c_str(),
                mon.last_error() ? mon.last_error().message().c_str() : "none");

    // 7. a port that accepts but sends nothing -> no_packets; a closed port -> refused
    asio::ip::tcp::acceptor silent(io, {asio::ip::make_address("127.0.0.1"), 0});
    asio::ip::tcp::socket accepted(io);
    silent.async_accept(accepted, [](std::error_code) {});
    ec::Monitor mute(io, "127.0.0.1", silent.local_endpoint().port(), true, 500ms);
    std::printf("7. silent port -> %s\n", (co_await mute.connect()).message().c_str());
    ec::Monitor refused(io, "127.0.0.1", 1, true, 500ms);
    std::printf("7. closed port -> %s\n", (co_await refused.connect()).message().c_str());
    rpc.close();
}

int main() {
    asio::io_context io;
    asio::co_spawn(io, tests(io), asio::detached);
    io.run();
}
