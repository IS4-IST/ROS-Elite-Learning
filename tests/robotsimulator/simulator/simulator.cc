// Simulated EC66: a MuJoCo arm behind the real controller's network interface.
//
//   ./simulator [model.xml] [--headless] [--cold] [--no-collision] [--bind 127.0.0.1]
//
//   --cold          start like a freshly booted robot: teach mode, brakes closed
//   --bind ADDR     listen address (default 127.0.0.1; 0.0.0.0 to reach it from other machines)
//   --headless      no window (servers + physics only)
//
// Port 8055 answers JSON-RPC and port 8056 streams 1024-byte monitor packets every
// 8 ms, so ec_robot, the bridge and the digital twin connect to it exactly as to
// the robot at 192.168.0.11. The ImGui "teach pendant" panel does what you'd do on
// the real pendant: switch teach/remote, press the e-stop, release the brakes.
//
// Two io_contexts, each driven by self re-arming timers:
//   control (own thread)   8055 / 8056 servers + the 8 ms control timer:
//                          motion -> physics catch-up to wall time -> monitor packet
//   rendering (main)       ImGui pendant + MuJoCo scene at the display refresh rate
// The control cycle never waits for a frame, so a slow frame or a window resize
// can't make the monitor stream go stale for clients.

#include <GLFW/glfw3.h>
#include <imgui.h>
#include "asio.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <atomic>
#include <thread>

#include "common.h"
#include "viewer.h"
#include "emulator/controller.h"
#include "emulator/servers.h"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {

const char* kStates[] = {"stop", "pause", "e-stop", "running", "alarm", "collision"};

ImVec4 state_colour(int state) {
    switch (state) {
        case 0: return {0.6f, 0.9f, 0.6f, 1};    // stop
        case 3: return {0.4f, 0.7f, 1.0f, 1};    // running
        case 1: return {1.0f, 0.9f, 0.4f, 1};    // pause
        default: return {1.0f, 0.35f, 0.3f, 1};  // e-stop, alarm, collision
    }
}

struct Network {                          // written by the control thread, read by the GUI
    std::atomic<bool> rpc_connected{false};
    std::atomic<std::size_t> monitor_clients{0};
};

void pendant_panel(emu::Controller& controller, const Network& net, const mjModel* m, mjData* d) {
    const auto p = controller.pendant();
    double q[6] = {}, cmd[6] = {}, time = 0;
    {
        auto lock = controller.lock();
        for (int i = 0; i < m->nu && i < 6; ++i) {
            q[i] = d->qpos[m->jnt_qposadr[i]] * 180.0 / mjPI;
            cmd[i] = d->ctrl[i] * 180.0 / mjPI;
        }
        time = d->time;
    }

    ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({380, 0}, ImGuiCond_Always);
    ImGui::Begin("Teach pendant", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("mode");
    ImGui::SameLine(110);
    if (ImGui::RadioButton("teach", p.mode == 0) && p.mode != 0) controller.toggle_mode();
    ImGui::SameLine();
    if (ImGui::RadioButton("remote", p.mode == 2) && p.mode != 2) controller.toggle_mode();

    ImGui::Text("state");
    ImGui::SameLine(110);
    ImGui::TextColored(state_colour(p.state), "%s", kStates[p.state]);

    ImGui::Text("servos");
    ImGui::SameLine(110);
    ImGui::Text("%s   synced: %s   tool: %d", p.servo ? "on" : "off", p.synced ? "yes" : "no", p.tool);
    ImGui::Text("speed");
    ImGui::SameLine(110);
    ImGui::Text("%.0f %%", p.speed_override);

    bool brakes = p.brakes_released;
    if (ImGui::Checkbox("brakes released", &brakes)) controller.release_brakes(brakes);

    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Button, p.estop ? ImVec4(0.5f, 0.5f, 0.5f, 1) : ImVec4(0.8f, 0.1f, 0.1f, 1));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, p.estop ? ImVec4(0.6f, 0.6f, 0.6f, 1) : ImVec4(0.95f, 0.15f, 0.15f, 1));
    if (ImGui::Button(p.estop ? "release e-stop" : "E-STOP", {-1, 40})) controller.set_estop(!p.estop);
    ImGui::PopStyleColor(2);
    if (p.state == 4 || p.state == 5)
        ImGui::TextWrapped("Alarm: the client clears it with clearAlarm (ec_robot's power_on does this).");

    ImGui::SeparatorText("network");
    ImGui::Text("8055 client: %s", net.rpc_connected ? "connected" : "-");
    ImGui::Text("8056 clients: %zu", net.monitor_clients.load());

    ImGui::SeparatorText("joints");
    if (ImGui::BeginTable("joints", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableSetupColumn("joint");
        ImGui::TableSetupColumn("q (deg)");
        ImGui::TableSetupColumn("command (deg)");
        ImGui::TableHeadersRow();
        for (int i = 0; i < m->nu && i < 6; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%d", i + 1);
            ImGui::TableNextColumn(); ImGui::Text("%8.3f", q[i]);
            ImGui::TableNextColumn(); ImGui::Text("%8.3f", cmd[i]);
        }
        ImGui::EndTable();
    }
    ImGui::Text("sim time %.2f s", time);
    ImGui::TextDisabled("c: cycle cameras   esc: quit");
    ImGui::End();
}

}  // namespace

int main(int argc, char** argv) {
    std::string path = ec::default_model_path();
    std::string bind = "127.0.0.1";
    bool headless = false;
    emu::ControllerOptions opts;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless") headless = true;
        else if (a == "--cold") opts.cold_start = true;
        else if (a == "--no-collision") opts.collision_alarm = false;
        else if (a == "--bind" && i + 1 < argc) bind = argv[++i];
        else if (a[0] != '-') path = a;
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 1; }
    }

    try {
        ec::ModelPtr model = ec::load_model(path);
        ec::DataPtr data(mj_makeData(model.get()));
        ec::reset_home(model.get(), data.get());
        emu::Controller controller(model.get(), data.get(), opts);

        asio::io_context control_context;
        emu::RpcServer rpc(control_context, controller, bind, 8055);
        emu::MonitorServer monitor(control_context, bind, 8056);
        Network net;
        std::printf("EC66 simulator on %s:8055/8056 (%s)\n%s\n", bind.c_str(), path.c_str(),
                    controller.describe().c_str());

        asio::high_resolution_timer control_timer{control_context};
        const auto control_period = 8ms;
        const auto t0 = Clock::now();
        const double sim_t0 = data->time;
        auto last = t0;

        std::function<void()> control;
        control = [&]() {
            const auto now = Clock::now();
            const double dt = std::chrono::duration<double>(now - last).count();
            last = now;
            // physics catches up with wall time inside tick(), whatever dt turned out to be
            controller.tick(dt, sim_t0 + std::chrono::duration<double>(now - t0).count());
            monitor.broadcast(controller.status_packet());
            net.rpc_connected = rpc.connected();
            net.monitor_clients = monitor.clients();

            control_timer.expires_at(control_timer.expiry() + control_period);
            if (control_timer.expiry() < asio::high_resolution_timer::clock_type::now() - 100ms)
                control_timer.expires_after(control_period);   // after a long stall, don't fire a burst
            control_timer.async_wait([&](const asio::error_code& ec) {
                if (!ec) control();
            });
        };
        control_timer.expires_after(0ms);
        asio::post(control_context, control);

        if (headless) {
            asio::signal_set signals(control_context, SIGINT, SIGTERM);
            signals.async_wait([&](const asio::error_code&, int) { control_context.stop(); });
            control_context.run();
            return 0;
        }

        ec::Viewer viewer(model.get(), "EC66 simulator");   // may throw: before the thread starts
        std::thread control_thread([&] { control_context.run(); });
        asio::io_context context;
        asio::high_resolution_timer rendering_timer{context};
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        const auto rendering_period = std::chrono::duration_cast<asio::high_resolution_timer::clock_type::duration>(
            std::chrono::duration<double>(1.0 / (mode ? mode->refreshRate : 60)));

        std::function<void()> rendering;
        rendering = [&]() {
            viewer.begin_frame();
            pendant_panel(controller, net, model.get(), data.get());
            {
                auto lock = controller.lock();
                viewer.update_scene(data.get());
            }
            viewer.end_frame();

            if (viewer.should_close()) return;
            rendering_timer.expires_at(rendering_timer.expiry() + rendering_period);
            if (rendering_timer.expiry() < asio::high_resolution_timer::clock_type::now())
                rendering_timer.expires_after(rendering_period);   // skip frames we're late for
            rendering_timer.async_wait([&](const asio::error_code& ec) {
                if (!ec) rendering();
            });
        };
        rendering_timer.expires_after(0ms);
        asio::post(context, rendering);

        while (!viewer.should_close()) {
            context.run_one();
        }
        control_context.stop();
        control_thread.join();
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
