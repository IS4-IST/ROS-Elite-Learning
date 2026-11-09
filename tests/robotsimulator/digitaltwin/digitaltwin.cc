// Digital twin of the EC66: the MuJoCo model mirrors what the robot reports.
//
//   ./digitaltwin [model.xml] [--ip 192.168.0.11] [--max-change 30]
//
//   --max-change DEG   largest joint change one "move joint" may command (default 30)
//
// Mirroring only needs the monitor port (8056), which serves several clients, so the
// twin can run next to the bridge or any script. Every monitor packet's joint angles
// are copied into the model (kinematics only, no physics): what you see is what the
// controller says the arm is doing.
//
// The "Commands" panel can also drive the robot through ec_robot, over port 8055.
// The controller accepts only ONE 8055 client, so connecting here locks out the bridge
// and your scripts until you disconnect. Commands go through two locks:
//   1. connect to 8055                  (button)
//   2. arm commands                     (checkbox; drops by itself when the robot leaves
//                                        remote mode, the monitor goes stale or 8055 closes)
// STOP (button or space bar) works whenever 8055 is connected, armed or not.
// The green ghost shows the joint target before you send it. A move whose largest
// joint change exceeds --max-change is refused: one slip of a slider can mean 300 deg.
//
// Point it at the simulator with --ip 127.0.0.1 to try the panel without the arm.

#include <GLFW/glfw3.h>
#include <imgui.h>
#include "asio.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "common.h"
#include "viewer.h"
#include "ec_robot/monitor.hpp"
#include "ec_robot/robot.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {

constexpr double kDeg = mjPI / 180.0;

// Runs blocking client calls (connect, power on, ...) off the rendering thread.
// One job at a time; the result goes to message().
class Worker {
public:
    Worker() : thread_([this] { io_.run(); }) {}
    ~Worker() {
        guard_.reset();
        io_.stop();
        thread_.join();
    }

    bool busy() const { return busy_; }

    bool run(std::string what, std::function<void()> fn) {
        if (busy_.exchange(true)) return false;
        asio::post(io_, [this, what = std::move(what), fn = std::move(fn)] {
            try {
                fn();
                report(what + ": ok", false);
            } catch (const std::exception& e) {
                report(what + ": " + e.what(), true);
            }
            busy_ = false;
        });
        return true;
    }

    std::pair<std::string, bool> message() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return {message_, error_};
    }

private:
    void report(std::string text, bool error) {
        std::printf("%s\n", text.c_str());
        std::lock_guard<std::mutex> lock(mutex_);
        message_ = std::move(text);
        error_ = error;
    }

    asio::io_context io_;
    asio::executor_work_guard<asio::io_context::executor_type> guard_{io_.get_executor()};
    std::atomic<bool> busy_{false};
    mutable std::mutex mutex_;
    std::string message_;
    bool error_ = false;
    std::thread thread_;   // last: starts after everything above exists
};

// The connections, swapped by the worker and read by the rendering loop
struct Link {
    std::mutex mutex;
    std::shared_ptr<ec::Monitor> monitor;   // 8056, for mirroring
    std::shared_ptr<ec::Robot> robot;       // 8055 + its own 8056, for commands

    std::shared_ptr<ec::Monitor> get_monitor() { std::lock_guard<std::mutex> l(mutex); return monitor; }
    std::shared_ptr<ec::Robot> get_robot() { std::lock_guard<std::mutex> l(mutex); return robot; }
    void set_monitor(std::shared_ptr<ec::Monitor> m) { std::lock_guard<std::mutex> l(mutex); monitor = std::move(m); }
    void set_robot(std::shared_ptr<ec::Robot> r) { std::lock_guard<std::mutex> l(mutex); robot = std::move(r); }
};

struct Ui {
    bool armed = false;
    bool show_ghost = true;
    bool target_set = false;
    ec::Joints target{};          // deg
    double move_speed = 10;       // moveByJoint "speed"
    double speed_override = 20;   // setSpeed, %
    int tool = 1;
    double max_change = 30;       // deg, per "move joint"
    // packet rate
    std::uint64_t packets = 0;
    Clock::time_point rate_t = Clock::now();
    double rate = 0;
};

ImVec4 state_colour(ec::State s) {
    switch (s) {
        case ec::State::Stop: return {0.6f, 0.9f, 0.6f, 1};
        case ec::State::Running: return {0.4f, 0.7f, 1.0f, 1};
        case ec::State::Pause: return {1.0f, 0.9f, 0.4f, 1};
        default: return {1.0f, 0.35f, 0.3f, 1};
    }
}
const ImVec4 kRed{1.0f, 0.35f, 0.3f, 1};

double max_abs_diff(const ec::Joints& a, const ec::Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

// --- panels -------------------------------------------------------------------------
void observed_panel(const std::string& ip, const std::optional<ec::RobotStatus>& s, bool fresh, Ui& ui,
                    const ec::Monitor* monitor, Worker& jobs, const std::function<void()>& reconnect,
                    const mjModel* m, const mjData* d, int tcp_site) {
    ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({470, 0}, ImGuiCond_Always);
    ImGui::Begin("Robot (observed)", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("%s:8056", ip.c_str());
    ImGui::SameLine();
    if (!s) {
        ImGui::TextColored(kRed, jobs.busy() ? "connecting..." : "no data");
    } else if (!fresh) {
        ImGui::TextColored(kRed, "STALE (%.1f s old)", s->age_seconds());
    } else {
        ImGui::Text("%.0f packets/s, %.0f ms old", ui.rate, s->age_seconds() * 1000);
    }
    if (monitor && monitor->invalid_packets() > 0) {
        ImGui::SameLine();
        ImGui::TextColored(kRed, "%llu invalid", static_cast<unsigned long long>(monitor->invalid_packets()));
    }
    if ((!s || !fresh) && !jobs.busy()) {
        ImGui::SameLine();
        if (ImGui::SmallButton("reconnect")) reconnect();
    }
    if (!s) {
        ImGui::End();
        return;
    }

    ImGui::Text("mode %s   state", ec::to_string(s->robot_mode));
    ImGui::SameLine();
    ImGui::TextColored(state_colour(s->robot_state), "%s", ec::to_string(s->robot_state));
    ImGui::Text("servos %s   synced %s   e-stop %s   collision %s", s->servo_ready ? "on" : "off",
                s->motor_synced ? "yes" : "no", s->estop ? "PRESSED" : "no", s->collision ? "YES" : "no");

    if (ImGui::BeginTable("joints", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableSetupColumn("joint");
        ImGui::TableSetupColumn("q (deg)");
        ImGui::TableSetupColumn("qd (deg/s)");
        ImGui::TableSetupColumn("torque (Nm)");
        ImGui::TableSetupColumn("temp (C)");
        ImGui::TableHeadersRow();
        for (int i = 0; i < 6; ++i) {
            const double q = s->joint_pos[i];
            const bool out = m->jnt_limited[i] &&
                             (q * kDeg < m->jnt_range[2 * i] || q * kDeg > m->jnt_range[2 * i + 1]);
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%d", i + 1);
            ImGui::TableNextColumn();
            if (out) ImGui::TextColored(kRed, "%8.3f", q);   // outside the model's joint range
            else ImGui::Text("%8.3f", q);
            ImGui::TableNextColumn(); ImGui::Text("%7.2f", s->joint_speed[i]);
            ImGui::TableNextColumn(); ImGui::Text("%7.2f", s->joint_torque[i]);
            ImGui::TableNextColumn(); ImGui::Text("%5.1f", s->joint_temperature[i]);
        }
        ImGui::EndTable();
    }

    const auto& p = s->tcp_pose;
    ImGui::Text("TCP  %.1f %.1f %.1f mm   %.3f %.3f %.3f rad", p[0], p[1], p[2], p[3], p[4], p[5]);
    ImGui::Text("TCP speed %.1f mm/s", s->tcp_speed);

    // Model check: MuJoCo's flange (site "tcp") against the flange pose the controller reports
    if (tcp_site >= 0) {
        const double* x = d->site_xpos + 3 * tcp_site;
        const auto& f = s->flange_pose;
        const double err = std::sqrt(std::pow(x[0] * 1000 - f[0], 2) + std::pow(x[1] * 1000 - f[1], 2) +
                                     std::pow(x[2] * 1000 - f[2], 2));
        ImGui::Text("model vs controller flange:");
        ImGui::SameLine();
        if (err > 2.0) ImGui::TextColored(kRed, "%.1f mm", err);
        else ImGui::Text("%.2f mm", err);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Forward kinematics of the MuJoCo model at the reported joints, compared with\n"
                              "the flange position in the monitor packet. Large = model or zero offsets wrong.");
    }
    ImGui::TextDisabled("c: cycle cameras   esc: quit");
    ImGui::End();
}

void show_message(const Worker& w) {
    auto [text, error] = w.message();
    if (text.empty()) return;
    ImGui::Separator();
    if (error) ImGui::TextColored(kRed, "%s", text.c_str());
    else ImGui::TextWrapped("%s", text.c_str());
}

void commands_panel(const std::string& ip, const std::optional<ec::RobotStatus>& s, bool fresh, Ui& ui, Link& link,
                    Worker& jobs, Worker& urgent, const mjModel* m) {
    ImGui::SetNextWindowPos({10, 300}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({470, 0}, ImGuiCond_Always);
    ImGui::Begin("Commands", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    auto robot = link.get_robot();
    const bool remote = s && s->robot_mode == ec::Mode::Remote;
    if (ui.armed && (!robot || !fresh || !remote)) ui.armed = false;   // drop the arm switch by itself

    // 1. connection --------------------------------------------------------------
    ImGui::BeginDisabled(jobs.busy());
    if (!robot) {
        if (ImGui::Button("connect 8055")) {
            jobs.run("connect " + ip + ":8055", [&link, ip] {
                auto r = std::make_shared<ec::Robot>(ip);
                r->connect();
                link.set_robot(std::move(r));
            });
        }
        ImGui::SameLine();
        ImGui::TextWrapped("The controller takes one 8055 client: close the bridge and scripts first.");
    } else {
        if (ImGui::Button("disconnect 8055")) {
            ui.armed = false;
            jobs.run("disconnect 8055", [&link] { link.set_robot(nullptr); });
        }
    }
    ImGui::EndDisabled();
    if (!robot) {
        show_message(jobs);
        ImGui::End();
        return;
    }

    // STOP: always available while connected
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.1f, 0.1f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.15f, 0.15f, 1));
    const bool stop_key = ImGui::IsKeyPressed(ImGuiKey_Space, false) && !ImGui::GetIO().WantTextInput;
    if (ImGui::Button("STOP  (space)", {-1, 40}) || stop_key)
        urgent.run("stop", [robot] { robot->stop(); });
    ImGui::PopStyleColor(2);

    // 2. arm ----------------------------------------------------------------------
    ImGui::BeginDisabled(!fresh || !remote);
    ImGui::Checkbox("arm commands", &ui.armed);
    ImGui::EndDisabled();
    if (!remote) {
        ImGui::SameLine();
        ImGui::TextColored(kRed, "robot not in remote mode");
    }

    ImGui::BeginDisabled(!ui.armed || jobs.busy());
    ImGui::SetNextItemWidth(80);
    ImGui::InputInt("tool", &ui.tool);
    ui.tool = std::clamp(ui.tool, 0, 7);
    ImGui::SameLine();
    if (ImGui::Button("power on")) {
        const int tool = ui.tool;
        const double speed = ui.speed_override;
        jobs.run("power on", [robot, tool, speed] {
            robot->set_tool(tool);
            robot->power_on();
            robot->set_speed(speed);
        });
    }
    ImGui::SameLine();
    if (ImGui::Button("power off")) jobs.run("power off", [robot] { robot->power_off(); });
    ImGui::SameLine();
    if (ImGui::Button("clear alarm")) jobs.run("clear alarm", [robot] { robot->rpc().call("clearAlarm"); });

    ImGui::SetNextItemWidth(200);
    const double one = 1, hundred = 100;
    ImGui::SliderScalar("##override", ImGuiDataType_Double, &ui.speed_override, &one, &hundred, "%.0f %%");
    ImGui::SameLine();
    if (ImGui::Button("set speed override")) {
        const double v = ui.speed_override;
        jobs.run("set speed", [robot, v] { robot->set_speed(v); });
    }
    ImGui::EndDisabled();

    // joint target (editable while disarmed, so you can line up the ghost first)
    ImGui::SeparatorText("joint target");
    for (int i = 0; i < 6; ++i) {
        double lo = -360, hi = 360;
        if (m->jnt_limited[i]) { lo = m->jnt_range[2 * i] / kDeg; hi = m->jnt_range[2 * i + 1] / kDeg; }
        ImGui::PushID(i);
        ImGui::SetNextItemWidth(-90);
        ImGui::SliderScalar("##q", ImGuiDataType_Double, &ui.target[i], &lo, &hi, "%.2f deg");
        ImGui::SameLine();
        if (ImGui::SmallButton("-")) ui.target[i] -= 1;
        ImGui::SameLine();
        if (ImGui::SmallButton("+")) ui.target[i] += 1;
        ImGui::SameLine();
        ImGui::Text("J%d", i + 1);
        ImGui::PopID();
    }
    if (ImGui::Button("copy current") && s) ui.target = s->q();
    ImGui::SameLine();
    ImGui::Checkbox("show ghost", &ui.show_ghost);

    const double change = s ? max_abs_diff(ui.target, s->q()) : 0;
    const bool too_far = change > ui.max_change;
    ImGui::Text("largest joint change:");
    ImGui::SameLine();
    if (too_far) ImGui::TextColored(kRed, "%.1f deg  (limit %.0f: move refused)", change, ui.max_change);
    else ImGui::Text("%.1f deg  (limit %.0f)", change, ui.max_change);

    ImGui::BeginDisabled(!ui.armed || jobs.busy());
    ImGui::SetNextItemWidth(150);
    const double lo_v = 1, hi_v = 100;
    ImGui::SliderScalar("speed##move", ImGuiDataType_Double, &ui.move_speed, &lo_v, &hi_v, "%.0f");
    ImGui::SameLine();
    ImGui::BeginDisabled(too_far);
    const bool move = ImGui::Button("move joint");
    ImGui::EndDisabled();
    if (move && !too_far) {
        const ec::Joints q = ui.target;
        const double v = ui.move_speed;
        jobs.run("moveByJoint", [robot, q, v] { robot->move_joint(q, v, false); });
    }
    ImGui::EndDisabled();

    // last results
    show_message(jobs);
    if (urgent.message().second) show_message(urgent);   // only a failed stop is worth showing
    ImGui::End();
}

}  // namespace

int main(int argc, char** argv) {
    std::string path = ec::default_model_path();
    std::string ip = "192.168.0.11";
    double max_change = 30;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--ip" && i + 1 < argc) ip = argv[++i];
        else if (a == "--max-change" && i + 1 < argc) max_change = std::stod(argv[++i]);
        else if (a[0] != '-') path = a;
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 1; }
    }

    try {
        ec::ModelPtr model = ec::load_model(path);
        ec::DataPtr data(mj_makeData(model.get()));    // the observed robot
        ec::DataPtr ghost(mj_makeData(model.get()));   // the command target
        ec::reset_home(model.get(), data.get());
        mj_forward(model.get(), data.get());
        const int tcp_site = mj_name2id(model.get(), mjOBJ_SITE, "tcp");

        Link link;
        Worker jobs, urgent;
        Ui ui;
        ui.max_change = max_change;
        auto connect_monitor = [&] {
            jobs.run("monitor " + ip + ":8056", [&link, ip] {
                link.set_monitor(nullptr);
                auto mon = std::make_shared<ec::Monitor>(ip);
                mon->start(2000ms);
                link.set_monitor(std::move(mon));
            });
        };
        connect_monitor();

        ec::Viewer viewer(model.get(), ("EC66 digital twin - " + ip).c_str());

        asio::io_context context;
        asio::high_resolution_timer rendering_timer{context};
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        const auto rendering_period = std::chrono::duration_cast<asio::high_resolution_timer::clock_type::duration>(
            std::chrono::duration<double>(1.0 / (mode ? mode->refreshRate : 60)));

        std::function<void()> rendering;
        rendering = [&]() {
            // mirror the latest monitor packet
            auto monitor = link.get_monitor();
            std::optional<ec::RobotStatus> status;
            if (monitor) {
                try { status = monitor->latest(0ms); } catch (const std::exception&) {}
            }
            const bool fresh = status && status->age_seconds() < 0.2;
            if (status) {
                for (int i = 0; i < 6; ++i) data->qpos[model->jnt_qposadr[i]] = status->joint_pos[i] * kDeg;
                mj_forward(model.get(), data.get());
                if (!ui.target_set) { ui.target = status->q(); ui.target_set = true; }
            }
            if (monitor) {
                const auto now = Clock::now();
                const double dt = std::chrono::duration<double>(now - ui.rate_t).count();
                if (dt >= 1.0) {
                    ui.rate = (monitor->packets() - ui.packets) / dt;
                    ui.packets = monitor->packets();
                    ui.rate_t = now;
                }
            }

            viewer.begin_frame();
            observed_panel(ip, status, fresh, ui, monitor.get(), jobs, connect_monitor, model.get(), data.get(),
                           tcp_site);
            commands_panel(ip, status, fresh, ui, link, jobs, urgent, model.get());

            viewer.update_scene(data.get());
            if (ui.show_ghost && link.get_robot() && status && max_abs_diff(ui.target, status->q()) > 0.05) {
                for (int i = 0; i < 6; ++i) ghost->qpos[model->jnt_qposadr[i]] = ui.target[i] * kDeg;
                mj_forward(model.get(), ghost.get());
                viewer.add_ghost(ghost.get());
            }
            viewer.end_frame();

            if (viewer.should_close()) return;
            rendering_timer.expires_at(rendering_timer.expiry() + rendering_period);
            if (rendering_timer.expiry() < asio::high_resolution_timer::clock_type::now())
                rendering_timer.expires_after(rendering_period);
            rendering_timer.async_wait([&](const asio::error_code& ec) {
                if (!ec) rendering();
            });
        };
        rendering_timer.expires_after(0ms);
        asio::post(context, rendering);

        while (!viewer.should_close()) {
            context.run_one();
        }
        link.set_robot(nullptr);
        link.set_monitor(nullptr);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
