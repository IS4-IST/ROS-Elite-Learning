// Common types for the Elite EC client.
// Units are the controller's own: joints in degrees, poses [x, y, z, rx, ry, rz]
// in mm and radians, rotations fixed-axis X-Y-Z (R = Rz * Ry * Rx).
#pragma once

#include <array>
#include <chrono>
#include <stdexcept>
#include <string>

namespace ec {

using Clock = std::chrono::steady_clock;
using Joints = std::array<double, 6>;   // deg
using Pose = std::array<double, 6>;     // mm, rad

// Robot mode (manual 2.2.2.1, monitor robotMode)
enum class Mode : int { Teach = 0, Play = 1, Remote = 2 };
// Robot state (manual 2.2.2.2, monitor robotState)
enum class State : int { Stop = 0, Pause = 1, EStop = 2, Running = 3, Alarm = 4, Collision = 5 };

const char* to_string(Mode m);
const char* to_string(State s);

// Error reply from the controller, or a failed call
class ECError : public std::runtime_error {
public:
    ECError(const std::string& message, int code = 0, std::string method = {})
        : std::runtime_error(message), code_(code), method_(std::move(method)) {}
    int code() const { return code_; }
    const std::string& method() const { return method_; }

private:
    int code_;
    std::string method_;
};

// The robot is not in a state that allows the requested operation
class NotReady : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

}  // namespace ec
