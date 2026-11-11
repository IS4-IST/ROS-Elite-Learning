// Kinematics for the emulated controller, using the MuJoCo model.
//
// Conventions match the real controller (verified against V3.11.2 to < 1 mm):
//   joints in degrees, poses [x, y, z, rx, ry, rz] in mm and radians,
//   rotations fixed-axis X-Y-Z: R = Rz(rz) * Ry(ry) * Rx(rx).
// The TCP is the model's `tcp` site (the flange) plus the active tool's offset.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <optional>

namespace emu {

using Joints = std::array<double, 6>;   // deg
using Pose = std::array<double, 6>;     // mm, rad
using Mat3 = std::array<double, 9>;     // row-major

Mat3 rpy_to_mat(double rx, double ry, double rz);
void mat_to_rpy(const Mat3& R, double& rx, double& ry, double& rz);

class Kinematics {
public:
    explicit Kinematics(const mjModel* m);
    ~Kinematics();
    Kinematics(const Kinematics&) = delete;
    Kinematics& operator=(const Kinematics&) = delete;

    // Pose of the TCP for joints q (deg) with tool offset `tool` (flange frame, mm / rad)
    Pose forward(const Joints& q, const Pose& tool) const;
    // Pose of the flange (no tool)
    Pose flange(const Joints& q) const { return forward(q, Pose{}); }

    // Joints (deg) reaching `pose` with `tool`, starting from ref (the solution nearest ref).
    // Damped least squares on the site Jacobian, joint limits respected. nullopt if it doesn't converge.
    std::optional<Joints> inverse(const Pose& pose, const Pose& tool, const Joints& ref) const;

    bool within_limits(const Joints& q) const;

private:
    void site_pose(const Joints& q, double pos_m[3], Mat3& R) const;   // flange site, world frame
    const mjModel* m_;
    mjData* d_;            // scratch data, never stepped
    int site_;
};

}  // namespace emu
