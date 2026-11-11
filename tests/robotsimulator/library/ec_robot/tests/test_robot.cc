// The old test_against_mock checks (minus streaming), on the async Robot, plus new ones.
#include "ec_robot/robot.h"
#include <cmath>
#include <cstdio>
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

int failures = 0;
void check(const std::string& name, bool ok) {
    std::printf("%s  %s\n", ok ? "ok  " : "FAIL", name.c_str());
    if (!ok) ++failures;
}
double ms_since(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

// a smooth 3 s trajectory on joint 6: +delta and back to start
void make_trajectory(const ec::Joints& q0, double delta, int n, std::vector<double>& times, std::vector<ec::Joints>& points) {
    times.clear(); points.clear();
    for (int i = 0; i < n; ++i) {
        const double s = double(i) / (n - 1);
        ec::Joints q = q0;
        q[5] += delta * 0.5 * (1 - std::cos(2 * M_PI * s));
        times.push_back(3.0 * s);
        points.push_back(q);
    }
}

asio::awaitable<void> tests(asio::io_context& io, ec::Robot& robot) {
    std::error_code ec = co_await robot.connect();
    check("connect: " + (ec ? robot.last_error_detail() : std::string("8055 + 8056")), !ec);
    if (ec) co_return;
    const auto v = co_await robot.version();
    check("version: " + v.value, !v.ec);

    const ec::Reply bad = co_await robot.rpc().call("noSuchMethod");
    check("unknown method -> controller error " + std::to_string(bad.ec.value()), bad.ec.value() == -32601);

    ec::Joints q0 = robot.status()->q();
    ec = co_await robot.move_joint(q0, 10.0);
    check("move refused with servos off -> " + ec.message(), ec == ec::RobotErrc::servos_off);

    ec = co_await robot.power_on();
    check("power_on -> " + (ec ? robot.last_error_detail() : robot.describe()), !ec && robot.status()->servo_ready);

    ec = co_await robot.set_tool(1);
    const auto tool = co_await robot.tool_info(1);
    check("set_tool(1), tool_info: mass " + std::to_string(tool.value.mass) + " kg", !ec && !tool.ec);

    ec::Joints q1 = q0;
    q1[5] += 10;
    ec = co_await robot.move_joint(q1, 20.0);
    check("move_joint reaches target", !ec && std::abs(robot.status()->q()[5] - q1[5]) < 0.05);

    const auto fk = co_await robot.forward_kinematics(robot.status()->q());
    const auto ik = co_await robot.inverse_kinematics(fk.value, robot.status()->q());
    double err = 0;
    for (int i = 0; i < 6; ++i) err = std::max(err, std::abs(ik.value[i] - robot.status()->q()[i]));
    check("FK -> IK round trip, " + std::to_string(err) + " deg", !fk.ec && !ik.ec && err < 0.01);

    ec::Pose down = robot.status()->tcp_pose;
    down[2] -= 50;
    const auto refused = co_await robot.plan_line(down, 0.01);
    check("plan_line refuses beyond limit: " + robot.last_error_detail(), refused.ec == ec::RobotErrc::ik_jump);
    const auto line = co_await robot.move_line(down, 50.0);
    check("move_line 50 mm down: z = " + std::to_string(robot.status()->tcp_pose[2]),
          !line.ec && std::abs(robot.status()->tcp_pose[2] - down[2]) < 0.5);

    std::vector<double> times;
    std::vector<ec::Joints> points;
    make_trajectory(robot.status()->q(), 20, 3, times, points);
    times[2] = times[1];
    ec = co_await robot.run_joint_trajectory(times, points);
    check("bad timestamps rejected before sending -> " + ec.message(), ec == ec::RobotErrc::bad_trajectory);

    // trajectory, uploaded one request at a time (as before) vs 8 in flight
    for (int window : {1, 8}) {
        make_trajectory(robot.status()->q(), 20, 375, times, points);
        const auto t0 = Clock::now();
        ec = co_await robot.run_joint_trajectory(times, points, 100.0, 10.0, /*wait=*/false, 120s, nullptr, window);
        const double upload = ms_since(t0);
        const std::error_code done = co_await robot.wait_until_stopped(60s, points.back());
        check("trajectory, window " + std::to_string(window) + ": 375 points uploaded+started in " +
                  std::to_string(int(upload)) + " ms, ends at last point",
              !ec && !done && std::abs(robot.status()->q()[5] - points.back()[5]) < 0.05);
    }

    // cancel during the upload: the flag is set 30 ms in, the arm must not move
    make_trajectory(robot.status()->q(), 20, 2000, times, points);
    bool cancel = false;
    asio::steady_timer t(io);
    t.expires_after(30ms);
    t.async_wait([&cancel](std::error_code) { cancel = true; });
    const double before = robot.status()->q()[5];
    ec = co_await robot.run_joint_trajectory(times, points, 100.0, 10.0, true, 120s, &cancel, 1);
    t.expires_after(300ms);
    co_await t.async_wait(asio::use_awaitable);
    check("cancel during upload -> " + robot.last_error_detail() + ", arm moved " +
              std::to_string(std::abs(robot.status()->q()[5] - before)) + " deg",
          ec == ec::RobotErrc::cancelled && std::abs(robot.status()->q()[5] - before) < 0.01);

    // STOP while a long upload is in flight: its reply must not wait for the upload
    make_trajectory(robot.status()->q(), 20, 2000, times, points);
    double stop_ms = -1;
    asio::co_spawn(io, [&robot, &stop_ms, &t]() -> asio::awaitable<void> {
        t.expires_after(40ms);
        co_await t.async_wait(asio::use_awaitable);
        const auto s0 = Clock::now();
        co_await robot.stop();
        stop_ms = ms_since(s0);
    }, asio::detached);
    const auto u0 = Clock::now();
    ec = co_await robot.run_joint_trajectory(times, points, 100.0, 10.0, false, 120s, nullptr, 1);
    check("stop answered in " + std::to_string(stop_ms) + " ms while a " + std::to_string(int(ms_since(u0))) +
              " ms upload was running", stop_ms >= 0 && stop_ms < 50);
    co_await robot.stop_trajectory();

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
