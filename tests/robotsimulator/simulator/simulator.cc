// Simulated EC66: a MuJoCo arm behind the real controller's network interface.
//
//   ./simulator [model.xml] [--headless] [--cold] [--no-collision] [--bind 127.0.0.1]
//
//   --cold          start like a freshly booted robot: teach mode, brakes closed
//   --bind ADDR     listen address (default 127.0.0.1; 0.0.0.0 to reach it from other machines)
//   --headless      no window (servers + physics only; no cameras)
//   --camera NAME[:PORT]   stream the model's camera NAME as H.264 on PORT (default 5000,
//                   5001, ... in order). Repeat for more cameras. Needs the window:
//                   images are rendered with its OpenGL context.
//   --camera-size WxH      default 640x480 (at most the model's offwidth x offheight)
//   --camera-fps N         default 30
//   --camera-gop N         frames between keyframes, default 15
//   --camera-bitrate KBPS  default 2000
//   --camera-codec NAME    FFmpeg encoder, default libx264 (h264_nvenc, h264_vaapi, ...)
//
// Port 8055 answers JSON-RPC and port 8056 streams 1024-byte monitor packets every
// 8 ms, so ec_robot, the bridge and the digital twin connect to it exactly as to
// the robot at 192.168.0.11. The ImGui "teach pendant" panel does what you'd do on
// the real pendant: switch teach/remote, press the e-stop, release the brakes.
//
// Two io_contexts, each driven by self re-arming timers:
//   control (own thread)   8055 / 8056 servers + the 8 ms control timer:
//                          motion -> physics catch-up to wall time -> monitor packet
//   rendering (main)       ImGui pendant + MuJoCo scene at the display refresh rate,
//                          and the cameras: render offscreen -> H.264 -> TCP at their own rate
// The control cycle never waits for a frame, so a slow frame or a window resize
// can't make the monitor stream go stale for clients.

#include <GLFW/glfw3.h>
#include <imgui.h>
#include "asio.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <atomic>
#include <thread>
#include <vector>

#include "common.h"
#include "viewer.h"
#include "emulator/controller.h"
#include "emulator/servers.h"
#include "camera/encoder.h"
#include "camera/server.h"

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

struct CameraRequest {
    std::string name;
    int port = -1;   // -1: next free number from 5000
};

struct SimCamera {
    std::string name;
    int id = -1;
    camera::Encoder encoder;
    std::unique_ptr<camera::Server> server;
    std::vector<std::uint8_t> rgb;
    std::uint64_t frame_id = 0;
    double encode_ms = 0;   // smoothed render + encode time, for the panel
};

void camera_panel(const std::vector<std::unique_ptr<SimCamera>>& cameras, const camera::EncoderConfig& cfg) {
    ImGui::SetNextWindowPos({10, 520}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Cameras", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("%dx%d  %d fps  gop %d  %d kbit/s  %s", cfg.width, cfg.height, cfg.fps, cfg.gop, cfg.bitrate_kbps,
                cfg.codec.c_str());
    for (const auto& cam : cameras)
        ImGui::Text("%-12s :%u  clients %zu  %.1f ms/frame", cam->name.c_str(), cam->server->port(),
                    cam->server->clients(), cam->encode_ms);
    ImGui::End();
}

}  // namespace

int main(int argc, char** argv) {
    std::string path = ec::default_model_path();
    std::string bind = "127.0.0.1";
    bool headless = false;
    emu::ControllerOptions opts;
    std::vector<CameraRequest> camera_requests;
    int cam_width = 640, cam_height = 480, cam_fps = 30, cam_gop = 15, cam_kbps = 2000;
    std::string cam_codec = "libx264";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless") headless = true;
        else if (a == "--cold") opts.cold_start = true;
        else if (a == "--no-collision") opts.collision_alarm = false;
        else if (a == "--bind" && i + 1 < argc) bind = argv[++i];
        else if (a == "--camera" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const auto colon = spec.find(':');
            CameraRequest r;
            r.name = spec.substr(0, colon);
            if (colon != std::string::npos) r.port = std::atoi(spec.c_str() + colon + 1);
            camera_requests.push_back(r);
        }
        else if (a == "--camera-size" && i + 1 < argc) {
            if (std::sscanf(argv[++i], "%dx%d", &cam_width, &cam_height) != 2) {
                std::fprintf(stderr, "--camera-size wants WxH, e.g. 640x480\n");
                return 1;
            }
        }
        else if (a == "--camera-fps" && i + 1 < argc) cam_fps = std::atoi(argv[++i]);
        else if (a == "--camera-gop" && i + 1 < argc) cam_gop = std::atoi(argv[++i]);
        else if (a == "--camera-bitrate" && i + 1 < argc) cam_kbps = std::atoi(argv[++i]);
        else if (a == "--camera-codec" && i + 1 < argc) cam_codec = argv[++i];
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

        // --- control: the controller's 8 ms cycle ------------------------------------
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

        if (headless && camera_requests.empty()) {   // nothing to render: no window at all
            asio::signal_set signals(control_context, SIGINT, SIGTERM);
            signals.async_wait([&](const asio::error_code&, int) { control_context.stop(); });
            control_context.run();
            return 0;
        }

        ec::Viewer viewer(model.get(), "EC66 simulator", headless ? 64 : 1400, headless ? 64 : 900,
                          /*visible=*/!headless);   // may throw: before the thread starts
        std::thread control_thread([&] { control_context.run(); });
        asio::io_context context;
        asio::signal_set signals(context, SIGINT, SIGTERM);   // Ctrl+C: close like the window's button
        signals.async_wait([&](const asio::error_code& ec, int) {
            if (!ec) viewer.close();
        });

        // --- cameras: offscreen image -> H.264 -> TCP, on this thread (it owns the GL context)
        camera::EncoderConfig cam_cfg;
        cam_cfg.width = cam_width;
        cam_cfg.height = cam_height;
        cam_cfg.fps = cam_fps;
        cam_cfg.gop = cam_gop;
        cam_cfg.bitrate_kbps = cam_kbps;
        cam_cfg.codec = cam_codec;
        std::vector<std::unique_ptr<SimCamera>> cameras;
        int next_port = 5000;
        for (const CameraRequest& r : camera_requests) {
            auto cam = std::make_unique<SimCamera>();
            cam->name = r.name;
            cam->id = mj_name2id(model.get(), mjOBJ_CAMERA, r.name.c_str());
            if (cam->id < 0) {
                std::fprintf(stderr, "camera %s: not in the model (its cameras:", r.name.c_str());
                for (int c = 0; c < model->ncam; ++c) std::fprintf(stderr, " %s", mj_id2name(model.get(), mjOBJ_CAMERA, c));
                std::fprintf(stderr, ")\n");
                return 1;
            }
            if (cam_width > viewer.capture_max_width() || cam_height > viewer.capture_max_height()) {
                std::fprintf(stderr, "camera %s: %dx%d is larger than the model's offscreen buffer %dx%d "
                             "(<visual><global offwidth offheight>)\n", r.name.c_str(), cam_width, cam_height,
                             viewer.capture_max_width(), viewer.capture_max_height());
                return 1;
            }
            if (std::error_code ec = cam->encoder.open(cam_cfg)) {
                std::fprintf(stderr, "camera %s: %s (%s)\n", r.name.c_str(), ec.message().c_str(),
                             cam->encoder.error_detail().c_str());
                return 1;
            }
            const int port = r.port > 0 ? r.port : next_port;
            next_port = port + 1;
            cam->server = std::make_unique<camera::Server>(context);
            if (std::error_code ec = cam->server->listen(bind, static_cast<unsigned short>(port))) {
                std::fprintf(stderr, "camera %s: %s (%s)\n", r.name.c_str(), ec.message().c_str(),
                             cam->server->error_detail().c_str());
                return 1;
            }
            cam->rgb.resize(static_cast<std::size_t>(cam_width) * cam_height * 3);
            std::printf("camera %s: H.264 %dx%d@%d on %s:%d   (try: nc %s %d | ffplay -f h264 -)\n", r.name.c_str(),
                        cam_width, cam_height, cam_fps, bind.c_str(), port, bind.c_str(), port);
            cameras.push_back(std::move(cam));
        }

        asio::high_resolution_timer camera_timer{context};
        const auto camera_period = std::chrono::duration_cast<asio::high_resolution_timer::clock_type::duration>(
            std::chrono::duration<double>(1.0 / std::max(1, cam_fps)));
        std::function<void()> capture;
        capture = [&]() {
            for (auto& cam : cameras) {
                if (cam->server->clients() == 0) continue;   // nobody watching: don't render or encode
                const auto t0 = Clock::now();
                std::int64_t capture_ns;
                {
                    auto lock = controller.lock();
                    viewer.update_capture(data.get(), cam->id);
                    capture_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch()).count();
                }
                viewer.read_capture(cam_width, cam_height, cam->rgb.data());
                auto frame = std::make_shared<camera::EncodedFrame>();
                const camera::FrameStamp stamp{cam->frame_id, capture_ns};
                if (std::error_code ec = cam->encoder.encode(cam->rgb, /*bottom_up=*/true, stamp,
                                                             cam->server->wants_keyframe(), *frame)) {
                    std::fprintf(stderr, "camera %s: %s (%s)\n", cam->name.c_str(), ec.message().c_str(),
                                 cam->encoder.error_detail().c_str());
                    continue;
                }
                ++cam->frame_id;
                if (!frame->bytes.empty()) cam->server->broadcast(std::move(frame));
                const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                cam->encode_ms = cam->encode_ms == 0 ? ms : 0.9 * cam->encode_ms + 0.1 * ms;
            }
            camera_timer.expires_at(camera_timer.expiry() + camera_period);
            if (camera_timer.expiry() < asio::high_resolution_timer::clock_type::now())
                camera_timer.expires_after(camera_period);   // late: skip, don't burst
            camera_timer.async_wait([&](const asio::error_code& ec) {
                if (!ec) capture();
            });
        };
        if (!cameras.empty()) {
            camera_timer.expires_after(0ms);
            asio::post(context, capture);
        }
        asio::high_resolution_timer rendering_timer{context};
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        const auto rendering_period = std::chrono::duration_cast<asio::high_resolution_timer::clock_type::duration>(
            std::chrono::duration<double>(1.0 / (mode ? mode->refreshRate : 60)));

        std::function<void()> rendering;
        rendering = [&]() {
            viewer.begin_frame();
            pendant_panel(controller, net, model.get(), data.get());
            if (!cameras.empty()) camera_panel(cameras, cam_cfg);
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

        if (!headless) {   // hidden window: cameras only, no pendant or scene
            rendering_timer.expires_after(0ms);
            asio::post(context, rendering);
        }

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
