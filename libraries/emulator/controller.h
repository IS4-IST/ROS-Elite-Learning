// Emulated Elite EC controller on top of the MuJoCo simulation.
//
// Implements the controller's state machine and the JSON-RPC commands used by
// ec_robot / ec_robot_cpp (EC SDK-Socket Manual Ver3.19.2), driving the MuJoCo
// position servos. Behaviour follows what was observed on the lab's V3.11.2:
//  - results are JSON text (the version string therefore arrives double-encoded)
//  - encoder sync fails until the brakes are released ("pendant" key in the viewer)
//  - releasing the brakes also syncs the encoders
//  - moveByLine's target_pose form is rejected (it misbehaved on V3.11.2); use targetPos
//  - remote-only commands are refused outside remote mode
//
// Thread safety: every public method locks the controller's mutex; use lock() to
// read the simulation (e.g. for rendering) from another thread.
#pragma once

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "emulator/kinematics.h"
#include "emulator/motion.h"
#include "emulator/packet.h"

namespace emu {

struct ControllerOptions {
    bool cold_start = false;          // teach mode, brakes closed, encoders not synced
    bool collision_alarm = true;      // robot contacts with penetration -> collision state
    double max_joint_speed = 120.0;   // deg/s at 100 % (moveByJoint)
    double max_joint_acc = 360.0;     // deg/s^2
    std::string version = "V3.11.2-sim";
};

// A JSON-RPC command error, sent back to the client with its code
struct RpcError {
    int code;
    std::string message;
};

class Controller {
public:
    Controller(mjModel* m, mjData* d, ControllerOptions opts = {});

    // Handle one JSON-RPC request; returns the reply object, or nullopt when the
    // command is configured not to reply (streaming points, trajectory points).
    std::optional<nlohmann::json> handle_request(const nlohmann::json& request);

    // One control tick (call every 8 ms of wall time): run the active motion,
    // then step physics until simulated time catches up with `sim_time_target`.
    void tick(double dt, double sim_time_target);

    // Monitor packet for the current state
    std::vector<std::uint8_t> status_packet();

    // "Teach pendant" actions for the viewer / tests
    void toggle_mode();             // teach <-> remote
    void set_estop(bool pressed);
    void release_brakes(bool released);
    std::string describe();

    // Snapshot of the pendant-side state, for the simulator's GUI
    struct PendantView {
        int mode = 0;            // 0 teach, 1 play, 2 remote
        int state = 0;           // 0 stop, 1 pause, 2 e-stop, 3 running, 4 alarm, 5 collision
        bool servo = false, synced = false, brakes_released = false, estop = false;
        int tool = 0;
        double speed_override = 100;   // %
    };
    PendantView pendant();

    std::unique_lock<std::mutex> lock() { return std::unique_lock<std::mutex>(mutex_); }
    const mjModel* model() const { return m_; }
    mjData* data() { return d_; }

private:
    nlohmann::json dispatch(const std::string& method, const nlohmann::json& p, bool& reply);
    void require_remote(const std::string& method) const;
    Joints measured() const;               // deg
    Pose tool() const { return tools_[tool_].tcp; }
    void start_motion(std::unique_ptr<Motion> m);
    void cancel_motion();
    void raise(int state, const std::string& why);
    void check_collisions();

    mjModel* m_;
    mjData* d_;
    ControllerOptions opts_;
    Kinematics kin_;
    std::mutex mutex_;

    // state machine
    int mode_ = 2;              // 0 teach, 1 play, 2 remote
    int state_ = 0;             // 0 stop, 1 pause, 2 e-stop, 3 running, 4 alarm, 5 collision
    bool servo_ = false;
    bool synced_ = false;
    bool brakes_released_ = true;
    bool estop_ = false;
    double speed_override_ = 100.0;   // %

    struct Tool { Pose tcp{}; double mass = 0; std::array<double, 3> cog{}; };
    std::array<Tool, 8> tools_{};
    int tool_ = 0;

    // motion
    Joints cmd_{};                         // current joint command, deg
    std::unique_ptr<Motion> motion_;
    PassThrough* pass_through_ = nullptr;  // points into motion_ while streaming
    bool tt_reply_ = true;

    // trajectory upload
    struct Upload { std::size_t expected = 0; std::vector<double> times; std::vector<Joints> points; bool reply = true; };
    std::optional<Upload> upload_;
    std::optional<Upload> loaded_;         // validated trajectory, ready to run
    TrajectoryPlayback* playback_ = nullptr;

    Joints brake_q_{};                     // where the brakes hold the arm while the servos are off
    double settling_ = 0;                  // s spent waiting for the arm to reach the final command
    Joints last_q_{};                      // for joint speed / acceleration
    Joints last_qd_{};
    double last_tick_dt_ = 0.008;
    Pose last_tcp_{};
    double tcp_speed_ = 0;
};

}  // namespace emu
