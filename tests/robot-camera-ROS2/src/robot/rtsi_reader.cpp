#include "rtsi_reader.hpp"

#include <Elite/RtsiClientInterface.hpp>
#include <Elite/RtsiRecipe.hpp>

#include <exception>
#include <vector>

namespace robot {

namespace {
// RTSI output recipe. Names as used by the SDK's RtsiIOInterface getters.
const std::vector<std::string> kRecipe = {
    "timestamp",              // DOUBLE
    "actual_joint_positions", // VECTOR6D
    "actual_joint_speeds",    // VECTOR6D
    "actual_joint_torques",   // VECTOR6D
    "target_joint_positions", // VECTOR6D
    "actual_TCP_pose",        // VECTOR6D
    "actual_TCP_speed",       // VECTOR6D
    "actual_TCP_force",       // VECTOR6D
    "robot_mode",             // INT32
    "safety_status",          // INT32
    "speed_scaling",          // DOUBLE
};
}  // namespace

RtsiReader::RtsiReader(std::string robot_ip, double frequency_hz, SampleCallback on_sample,
                       StatusCallback on_status)
    : ip_(std::move(robot_ip)), frequency_(frequency_hz), on_sample_(std::move(on_sample)),
      on_status_(std::move(on_status))
{
}

RtsiReader::~RtsiReader() { stop(); }

void RtsiReader::start()
{
    if (running_.exchange(true)) return;
    thread_ = std::thread([this] { run(); });
}

void RtsiReader::stop()
{
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
}

bool RtsiReader::connect_once()
{
    try {
        client_ = std::make_unique<ELITE::RtsiClientInterface>();
        client_->connect(ip_);                                // port 30004
        if (!client_->isConnected()) throw std::runtime_error("cannot reach RTSI port 30004");
        if (!client_->negotiateProtocolVersion()) throw std::runtime_error("protocol version rejected");
        const std::string version = client_->getControllerVersion().toString();
        recipe_ = client_->setupOutputRecipe(kRecipe, frequency_);
        if (!recipe_) throw std::runtime_error("output recipe rejected");
        if (!client_->start()) throw std::runtime_error("start rejected");
        connected_ = true;
        if (on_status_) on_status_(true, "connected to " + ip_ + ", controller " + version);
        return true;
    } catch (const std::exception& e) {
        if (on_status_) on_status_(false, "connect to " + ip_ + " failed: " + e.what());
    }
    disconnect();
    return false;
}

void RtsiReader::disconnect()
{
    connected_ = false;
    if (client_) {
        try { client_->disconnect(); } catch (...) {}
    }
    recipe_.reset();
    client_.reset();
}

void RtsiReader::run()
{
    while (running_) {
        if (!client_) {
            if (!connect_once()) {
                for (int i = 0; i < 10 && running_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }

        try {
            // Blocks until the next RTSI data packet (1 s timeout inside the SDK).
            if (!client_->receiveData(recipe_, false)) {
                if (!client_->isConnected()) throw std::runtime_error("no data from robot");
                continue;                                      // some other packet type: ignore
            }
            RobotSample s;
            s.received = std::chrono::steady_clock::now();
            recipe_->getValue("timestamp", s.controller_time);
            recipe_->getValue("actual_joint_positions", s.q);
            recipe_->getValue("actual_joint_speeds", s.qd);
            recipe_->getValue("actual_joint_torques", s.tau);
            recipe_->getValue("target_joint_positions", s.q_target);
            recipe_->getValue("actual_TCP_pose", s.tcp_pose);
            recipe_->getValue("actual_TCP_speed", s.tcp_speed);
            recipe_->getValue("actual_TCP_force", s.tcp_force);
            recipe_->getValue("robot_mode", s.robot_mode);
            recipe_->getValue("safety_status", s.safety_mode);
            recipe_->getValue("speed_scaling", s.speed_scaling);
            on_sample_(s);
        } catch (const std::exception& e) {
            if (on_status_) on_status_(false, std::string("connection lost: ") + e.what());
            disconnect();
        }
    }
    disconnect();
}

const char* robot_mode_name(int32_t m)
{
    switch (m) {
        case -1: return "NO_CONTROLLER";
        case 0: return "DISCONNECTED";
        case 1: return "CONFIRM_SAFETY";
        case 2: return "BOOTING";
        case 3: return "POWER_OFF";
        case 4: return "POWER_ON";
        case 5: return "IDLE";
        case 6: return "BACKDRIVE";
        case 7: return "RUNNING";
        case 8: return "UPDATING_FIRMWARE";
        case 9: return "WAITING_CALIBRATION";
        default: return "UNKNOWN";
    }
}

const char* safety_mode_name(int32_t m)
{
    switch (m) {
        case 1: return "NORMAL";
        case 2: return "REDUCED";
        case 3: return "PROTECTIVE_STOP";
        case 4: return "RECOVERY";
        case 5: return "SAFEGUARD_STOP";
        case 6: return "SYSTEM_EMERGENCY_STOP";
        case 7: return "ROBOT_EMERGENCY_STOP";
        case 8: return "VIOLATION";
        default: return "UNKNOWN";
    }
}

}  // namespace robot
