// The old streaming checks on the async Streamer, plus timing, cross-thread targets, bad config.
#include "ec_robot/streamer.h"
#include <cmath>
#include <cstdio>
#include <thread>
using namespace std::chrono_literals;

int failures = 0;
void check(const std::string& name, bool ok) {
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", name.c_str());
    if (!ok) ++failures;
}

asio::awaitable<void> sleep_for(asio::io_context& io, std::chrono::milliseconds d) {
    asio::steady_timer t(io);
    t.expires_after(d);
    co_await t.async_wait(asio::use_awaitable);
}

asio::awaitable<void> tests(asio::io_context& io, ec::Robot& robot) {
    if (std::error_code ec = co_await robot.connect()) { check("connect: " + robot.last_error_detail(), false); co_return; }
    if (std::error_code ec = co_await robot.power_on()) { check("power_on: " + robot.last_error_detail(), false); co_return; }

    {   // bad configuration is refused before anything is sent
        ec::StreamerConfig cfg;
        cfg.period_ms = 1;
        ec::Streamer st(robot, cfg);
        const std::error_code ec = co_await st.start();
        check("period 1 ms refused -> " + ec.message(), ec == ec::StreamErrc::bad_config);
    }

    {   // rate limiter: a 90 deg jump moves at max_joint_speed only; then back, settle, stop
        ec::StreamerConfig cfg;
        cfg.lookahead_ms = 50;
        cfg.max_joint_speed = 20;
        cfg.max_tracking_error = 30;
        ec::Streamer st(robot, cfg);
        std::error_code ec = co_await st.start();
        check("start -> " + (ec ? ec.message() : std::string("ok")), !ec);
        ec::Joints far = st.command();
        const double start = far[0];
        far[0] += 90;
        st.set_target(far);
        const auto ticks0 = st.ticks();
        co_await sleep_for(io, 1000ms);
        const double moved = st.command()[0] - start;
        check("rate limiter: 90 deg jump -> " + std::to_string(moved) + " deg in 1 s", moved >= 15 && moved <= 21);
        const auto ticks = st.ticks() - ticks0;
        check("timing: " + std::to_string(ticks) + " ticks in 1 s (8 ms period -> 125)", ticks >= 120 && ticks <= 130);

        // a policy on another thread posts its target onto the robot's io_context
        ec::Joints back = far;
        back[0] = start;
        std::thread policy([&io, &st, back] { asio::post(io, [&st, back] { st.set_target(back); }); });
        co_await sleep_for(io, 20ms);   // let it finish first: join() blocks this thread, and the streamer runs on it
        policy.join();
        while (!st.at_target()) co_await sleep_for(io, 50ms);
        ec = co_await st.stop();
        const double end = robot.status()->q()[0];
        check("target from another thread reached, stop -> " + (ec ? ec.message() : std::string("ok")) +
                  ", arm at " + std::to_string(end) + " (start " + std::to_string(start) + ")",
              !ec && std::abs(end - start) < 0.1);
        check("overruns = " + std::to_string(st.overruns()), st.overruns() == 0);
    }

    {   // tracking error aborts the stream
        ec::StreamerConfig cfg;
        cfg.lookahead_ms = 50;
        cfg.max_tracking_error = 0.0001;
        ec::Streamer st(robot, cfg);
        co_await st.start();
        ec::Joints q = st.command();
        q[5] += 5;
        st.set_target(q);
        co_await sleep_for(io, 500ms);
        const bool aborted_alone = !st.running();
        const std::error_code ec = co_await st.stop();
        check("tracking error aborts the stream by itself: " + st.error_detail(),
              aborted_alone && ec == ec::StreamErrc::tracking_error);
        const std::error_code after = st.set_target(q);
        check("set_target after the abort -> " + after.message(), after == ec::StreamErrc::tracking_error);
    }

    std::printf("\n%s (%d failed)\n", failures ? "FAILURES" : "all tests passed", failures);
    robot.close();
}

int main() {
    asio::io_context io;
    ec::Robot robot(io, "127.0.0.1");
    asio::co_spawn(io, tests(io, robot), asio::detached);
    io.run();
    return failures != 0;
}
