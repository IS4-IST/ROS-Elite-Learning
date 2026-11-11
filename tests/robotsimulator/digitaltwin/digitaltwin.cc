// Digital twin of the EC66: the MuJoCo model mirrors what the robot reports, and the twin
// is the ONLY program that commands the robot.
//
//   ./digitaltwin [model.xml] [--ip 192.168.0.11] [--max-change 30] [--margin 10] [--no-ros]
//
//   --max-change DEG   largest joint change one "move joint" may command (default 30)
//   --margin MM        clearance kept by planning and checked on every motion (default 10)
//   --no-ros           don't start the ROS interface even if it was built
//
// Mirroring uses only the monitor port (8056), which serves several clients, so the twin
// can run next to anything. Every packet's joint angles go into the model (kinematics
// only, no physics): what you see is what the controller says the arm is doing.
//
// Commands go through twin::Core (twin_core.h), from two front ends:
//   - this GUI (the operator): connect 8055, "arm commands", then power, moves, plans
//   - ROS (when built with it, ros_interface.h): /follow_joint_trajectory, /stop, /power,
//     /clear_alarm. Only after the operator ticks "accept external commands"; while an
//     external motion runs, the operator's motion controls are locked.
// STOP (button or space bar) works for everyone, always, while 8055 is connected.
//
// Point it at the simulator with --ip 127.0.0.1 to try everything without the arm.

#include <GLFW/glfw3.h>
#include <imgui.h>
#include "asio.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "common.h"
#include "viewer.h"
#include "twin_core.h"
#ifdef EC_HAVE_ROS
#include "ros_env.h"
#include "ros_interface.h"
#endif

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using twin::Source;

namespace {

constexpr double kDeg = mjPI / 180.0;

struct Ui {
    bool show_ghost = true;
    bool target_set = false;
    ec::Joints target{};          // deg
    double move_speed = 10;       // moveByJoint "speed"
    double speed_override = 20;   // setSpeed, %
    int tool = 1;
    double max_change = 30;       // deg, per "move joint"
    double plan_speed = 30;       // deg/s, every joint (motion limits)
    double plan_acc = 60;         // deg/s^2
    bool plan_preview = true;
    std::string ros_text = "ROS: not built in";
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
const ImVec4 kGrey{0.6f, 0.6f, 0.6f, 1};

double max_abs_diff(const ec::Joints& a, const ec::Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

void show_result(const twin::View& v) {
    const twin::Outcome& o = v.last;
    if (o.message.empty()) return;
    ImGui::Separator();
    if (!o.ok) ImGui::TextColored(kRed, "%s", o.message.c_str());
    else ImGui::TextWrapped("%s", o.message.c_str());
}

// --- panels -------------------------------------------------------------------------
void observed_panel(twin::Core& core, const twin::View& v, bool fresh, Ui& ui,
                    const mjModel* m, const mjData* d, int tcp_site) {
    const std::optional<ec::RobotStatus>& s = v.status;
    ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({470, 0}, ImGuiCond_Always);
    ImGui::Begin("Robot (observed)", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("%s:8056", core.ip().c_str());
    ImGui::SameLine();
    if (!s) {
        ImGui::TextColored(kRed, v.connecting ? "connecting..." : "no data");
    } else if (!fresh) {
        ImGui::TextColored(kRed, "STALE (%.1f s old)", s->age_seconds());
    } else {
        ImGui::Text("%.0f packets/s, %.0f ms old", ui.rate, s->age_seconds() * 1000);
    }
    if (v.invalid_packets > 0) {
        ImGui::SameLine();
        ImGui::TextColored(kRed, "%llu invalid", static_cast<unsigned long long>(v.invalid_packets));
    }
    if ((!s || !fresh) && !v.connecting) {
        ImGui::SameLine();
        if (ImGui::SmallButton("reconnect")) core.connect_monitor();
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


void commands_panel(twin::Core& core, const twin::View& v, Ui& ui, const mjModel* m) {
    const std::optional<ec::RobotStatus>& s = v.status;
    ImGui::SetNextWindowPos({10, 300}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({470, 0}, ImGuiCond_Always);
    ImGui::Begin("Commands", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const bool connected = v.rpc_connected;
    const bool external_running = v.active == Source::External;

    // 1. connection --------------------------------------------------------------
    ImGui::BeginDisabled(v.busy || v.connecting || v.active != Source::None);
    if (!connected) {
        if (ImGui::Button("connect 8055")) core.connect_robot();
        ImGui::SameLine();
        ImGui::TextWrapped("The controller takes one 8055 client: close other programs that use it first.");
    } else if (ImGui::Button("disconnect 8055")) {
        core.disconnect_robot();
    }
    ImGui::EndDisabled();
    if (!connected) {
        show_result(v);
        ImGui::End();
        return;
    }

    // STOP: always available while connected
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.1f, 0.1f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.15f, 0.15f, 1));
    const bool stop_key = ImGui::IsKeyPressed(ImGuiKey_Space, false) && !ImGui::GetIO().WantTextInput;
    if (ImGui::Button("STOP  (space)", {-1, 40}) || stop_key) core.stop("the operator");
    ImGui::PopStyleColor(2);

    // 2. permissions ----------------------------------------------------------------
    bool armed = v.armed;
    ImGui::BeginDisabled(!armed && !v.can_arm);
    if (ImGui::Checkbox("arm commands", &armed)) core.set_armed(armed);
    ImGui::EndDisabled();
    if (s && s->robot_mode != ec::Mode::Remote) {
        ImGui::SameLine();
        ImGui::TextColored(kRed, "robot not in remote mode");
    }
    bool external = v.external;
    ImGui::BeginDisabled(!v.armed);
    if (ImGui::Checkbox("accept external commands", &external)) core.set_external_allowed(external);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Lets ROS clients (e.g. the planner) run motions, power on/off and clear alarms.\n"
                          "Unticking it during an external motion stops that motion.");
    ImGui::SameLine();
    ImGui::TextColored(kGrey, "%s", ui.ros_text.c_str());

    if (external_running) {
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.9f, 0.55f, 0.1f, 1));
        ImGui::ProgressBar(static_cast<float>(v.progress), {-1, 0}, "external motion running");
        ImGui::PopStyleColor();
        ImGui::TextWrapped("%s", v.motion_text.c_str());
        ImGui::TextColored(kGrey, "Your motion controls are locked. STOP, or untick \"accept external\", aborts it.");
    }

    // 3. operator commands ------------------------------------------------------------
    const bool locked = !v.armed || v.busy || v.active != Source::None;
    ImGui::BeginDisabled(locked);
    ImGui::SetNextItemWidth(80);
    ImGui::InputInt("tool", &ui.tool);
    ui.tool = std::clamp(ui.tool, 0, 7);
    ImGui::SameLine();
    if (ImGui::Button("power on")) core.power(true, Source::Operator);
    ImGui::SameLine();
    if (ImGui::Button("power off")) core.power(false, Source::Operator);
    ImGui::SameLine();
    if (ImGui::Button("clear alarm")) core.clear_alarm(Source::Operator);

    ImGui::SetNextItemWidth(200);
    const double one = 1, hundred = 100;
    ImGui::SliderScalar("##override", ImGuiDataType_Double, &ui.speed_override, &one, &hundred, "%.0f %%");
    ImGui::SameLine();
    if (ImGui::Button("set speed override")) core.set_speed(ui.speed_override);
    ImGui::EndDisabled();

    // joint target (editable while locked, so you can line up the ghost first)
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
    if (too_far) ImGui::TextColored(kRed, "%.1f deg  (limit %.0f: plan it instead)", change, ui.max_change);
    else ImGui::Text("%.1f deg  (limit %.0f)", change, ui.max_change);

    ImGui::BeginDisabled(locked);
    ImGui::SetNextItemWidth(150);
    const double lo_v = 1, hi_v = 100;
    ImGui::SliderScalar("speed##move", ImGuiDataType_Double, &ui.move_speed, &lo_v, &hi_v, "%.0f");
    ImGui::SameLine();
    ImGui::BeginDisabled(too_far);
    if (ImGui::Button("move joint") && !too_far) core.move_joint(ui.target, ui.move_speed);
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    show_result(v);
    ImGui::End();
}

void motion_panel(twin::Core& core, const twin::View& v, bool fresh, Ui& ui) {
    const std::optional<ec::RobotStatus>& s = v.status;
    ImGui::SetNextWindowPos({500, 10}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({420, 0}, ImGuiCond_Always);
    ImGui::Begin("Motion", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // limits: time the operator's plans and external paths, and cap external timed trajectories
    ImGui::TextColored(kGrey, "motion limits (every joint)");
    ImGui::SetNextItemWidth(120);
    const double v_lo = 1, v_hi = 120, a_lo = 1, a_hi = 240;
    ImGui::SliderScalar("deg/s", ImGuiDataType_Double, &ui.plan_speed, &v_lo, &v_hi, "%.0f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    ImGui::SliderScalar("deg/s^2", ImGuiDataType_Double, &ui.plan_acc, &a_lo, &a_hi, "%.0f");

    ImGui::BeginDisabled(!fresh || v.busy || v.active != Source::None);
    if (ImGui::Button("plan to target")) core.plan_to(ui.target);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("(target = the joint sliders)");

    const auto& plan = v.plan;
    ImGui::Separator();
    if (!plan) {
        ImGui::TextDisabled("no plan");
        ImGui::End();
        return;
    }
    ImGui::TextWrapped("%s", plan->source.c_str());
    ImGui::Text("%s", plan->summary.c_str());
    ImGui::Checkbox("preview (blue ghost)", &ui.plan_preview);
    ImGui::SameLine();
    if (ImGui::SmallButton("discard")) core.discard_plan();

    const double off = s ? max_abs_diff(s->q(), plan->traj.points.front()) : 1e9;
    const char* why_not = !v.rpc_connected ? "connect 8055 in Commands"
                        : !v.armed ? "arm commands in Commands"
                        : v.active == Source::External ? "an external motion is running"
                        : off > 0.5 ? "the arm is not at the plan's start: plan again"
                        : v.busy || v.active != Source::None ? "busy"
                        : nullptr;
    ImGui::BeginDisabled(why_not != nullptr);
    if (ImGui::Button("execute plan", {-1, 30}) && !why_not) {
        core.run_plan(plan, Source::Operator, {});   // a refusal shows up as the last result
    }
    ImGui::EndDisabled();
    if (why_not) ImGui::TextColored(off > 0.5 && s ? kRed : kGrey, "%s", why_not);
    show_result(v);
    ImGui::End();
}

// Where the preview ghost is: the plan replayed in real time, with a 1 s pause at the end
const ec::Joints& preview_pose(const twin::Plan& plan) {
    const double cycle = plan.traj.duration() + 1.0;
    const double t = std::min(std::fmod(std::chrono::duration<double>(Clock::now() - plan.created).count(), cycle),
                              plan.traj.duration());
    auto it = std::upper_bound(plan.traj.times.begin(), plan.traj.times.end(), t);
    const std::size_t i = it == plan.traj.times.begin() ? 0 : static_cast<std::size_t>(it - plan.traj.times.begin()) - 1;
    return plan.traj.points[i];
}

}  // namespace

int main(int argc, char** argv) {
#ifdef EC_HAVE_ROS
    ec::ensure_ros_environment(argv);   // first: may restart the program once
#endif
    std::string path = ec::default_model_path();
    std::string ip = "192.168.0.11";
    double max_change = 30, margin_mm = 10;
    bool no_ros = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--ip" && i + 1 < argc) ip = argv[++i];
        else if (a == "--max-change" && i + 1 < argc) max_change = std::stod(argv[++i]);
        else if (a == "--margin" && i + 1 < argc) margin_mm = std::stod(argv[++i]);
        else if (a == "--no-ros") no_ros = true;
        else if (a == "--ros-args") break;   // the rest belongs to ROS
        else if (a[0] != '-') path = a;
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 1; }
    }

    try {
        ec::ModelPtr model = ec::load_model(path);
        ec::DataPtr data(mj_makeData(model.get()));         // the observed robot
        ec::DataPtr ghost(mj_makeData(model.get()));        // the joint target
        ec::DataPtr plan_ghost(mj_makeData(model.get()));   // the plan preview
        ec::reset_home(model.get(), data.get());
        mj_forward(model.get(), data.get());
        const int tcp_site = mj_name2id(model.get(), mjOBJ_SITE, "tcp");

        ec::planning::Options plan_options;
        plan_options.margin = margin_mm / 1000.0;
        twin::Core core(model.get(), ip, plan_options);
        Ui ui;
        ui.max_change = max_change;

#ifdef EC_HAVE_ROS
        std::unique_ptr<twin::RosInterface> ros;   // after core: destroyed before it
        if (no_ros) {
            ui.ros_text = "ROS: off (--no-ros)";
        } else {
            ros = std::make_unique<twin::RosInterface>(core, argc, argv);
            ui.ros_text = "ROS: on";
        }
#else
        (void)no_ros;
#endif

        ec::Viewer viewer(model.get(), ("EC66 digital twin - " + ip).c_str());

        asio::io_context context;
        // Ctrl+C / SIGTERM: close the window like the close button (a running motion is stopped below)
        asio::signal_set signals(context, SIGINT, SIGTERM);
        signals.async_wait([&](const asio::error_code& ec, int) {
            if (!ec) viewer.close();
        });
        asio::high_resolution_timer rendering_timer{context};
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        const auto rendering_period = std::chrono::duration_cast<asio::high_resolution_timer::clock_type::duration>(
            std::chrono::duration<double>(1.0 / (mode ? mode->refreshRate : 60)));

        std::function<void()> rendering;
        rendering = [&]() {
            core.set_limits(ui.plan_speed, ui.plan_acc);
            core.set_power_settings(ui.tool, ui.speed_override);

            // mirror the latest monitor packet
            const twin::View view = core.view();   // one consistent snapshot per frame
            const std::optional<ec::RobotStatus>& status = view.status;
            const bool fresh = twin::Core::fresh(status);
            if (status) {
                for (int i = 0; i < 6; ++i) data->qpos[model->jnt_qposadr[i]] = status->joint_pos[i] * kDeg;
                mj_forward(model.get(), data.get());
                if (!ui.target_set) { ui.target = status->q(); ui.target_set = true; }
            }
            {
                const auto now = Clock::now();
                const double dt = std::chrono::duration<double>(now - ui.rate_t).count();
                if (dt >= 1.0) {
                    ui.rate = (view.packets - ui.packets) / dt;
                    ui.packets = view.packets;
                    ui.rate_t = now;
                }
            }

            viewer.begin_frame();
            observed_panel(core, view, fresh, ui, model.get(), data.get(), tcp_site);
            commands_panel(core, view, ui, model.get());
            motion_panel(core, view, fresh, ui);

            viewer.update_scene(data.get());
            if (ui.show_ghost && view.rpc_connected && status && max_abs_diff(ui.target, status->q()) > 0.05) {
                for (int i = 0; i < 6; ++i) ghost->qpos[model->jnt_qposadr[i]] = ui.target[i] * kDeg;
                mj_forward(model.get(), ghost.get());
                viewer.add_ghost(ghost.get());
            }
            // the plan, replayed in blue (hidden while the arm is moving: then watch the arm)
            if (const auto& plan = view.plan; plan && ui.plan_preview && status &&
                                         status->robot_state != ec::State::Running) {
                const ec::Joints& q = preview_pose(*plan);
                for (int i = 0; i < 6; ++i) plan_ghost->qpos[model->jnt_qposadr[i]] = q[i] * kDeg;
                mj_forward(model.get(), plan_ghost.get());
                viewer.add_ghost(plan_ghost.get(), {0.3f, 0.6f, 1.0f, 0.35f});
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
        // (a motion still running is stopped by the core's destructor)
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
