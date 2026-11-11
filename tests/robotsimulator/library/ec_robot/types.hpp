// Common types for the Elite EC client.
// Units are the controller's own: joints in degrees, poses [x, y, z, rx, ry, rz]
// in mm and radians, rotations fixed-axis X-Y-Z (R = Rz * Ry * Rx).
// Header-only vocabulary: no asio, no json, so anything can include it cheaply
// (the planning library does, for Joints).
#pragma once

#include <array>
#include <chrono>
#include <system_error>

namespace ec {

using Clock = std::chrono::steady_clock;
using Joints = std::array<double, 6>;   // deg
using Pose = std::array<double, 6>;     // mm, rad

// Robot mode (manual 2.2.2.1, monitor robotMode)
enum class Mode : int { Teach = 0, Play = 1, Remote = 2 };
// Robot state (manual 2.2.2.2, monitor robotState)
enum class State : int { Stop = 0, Pause = 1, EStop = 2, Running = 3, Alarm = 4, Collision = 5 };

const char* to_string(Mode m);    // types.cc
const char* to_string(State s);

// An error code plus a value, for operations that return data
template <typename T>
struct Result {
    std::error_code ec;
    T value{};
};

}  // namespace ec
