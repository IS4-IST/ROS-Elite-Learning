// Motion generators for the emulated controller. Each one produces the joint
// command (deg) for every control tick, the way the real controller turns a
// command into a stream of servo setpoints.
#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "emulator/kinematics.h"

namespace emu {

class Motion {
public:
    virtual ~Motion() = default;
    // Advance by dt seconds and update q (deg). Returns false when the motion is finished.
    // Sets `error` and returns false if the motion has to abort.
    virtual bool step(double dt, Joints& q) = 0;
    std::string error;
};

// Smooth 0 -> 1 profile with a trapezoidal velocity, duration from distance, speed and acceleration
class TrapezoidProfile {
public:
    TrapezoidProfile(double distance, double v_max, double a_max);
    double duration() const { return T_; }
    double s(double t) const;   // 0..1
private:
    double D_, v_, a_, ta_, T_;
};

// moveByJoint: all joints start and finish together
class JointMove : public Motion {
public:
    JointMove(const Joints& q0, const Joints& q1, double v_max_deg_s, double a_max_deg_s2);
    bool step(double dt, Joints& q) override;

private:
    Joints q0_, q1_;
    TrapezoidProfile profile_;
    double t_ = 0;
};

// moveByLine (joint form): straight line of the TCP from FK(q0) to FK(q1), orientation slerped
class LineMove : public Motion {
public:
    LineMove(const Kinematics& kin, const Pose& tool, const Joints& q0, const Joints& q1, double speed_mm_s);
    bool step(double dt, Joints& q) override;

private:
    const Kinematics& kin_;
    Pose tool_;
    Joints q1_;
    double p0_[3], p1_[3];
    double quat0_[4], quat1_[4];
    std::optional<TrapezoidProfile> profile_;
    double t_ = 0;
};

// Trajectory service playback: linear interpolation between timestamped points
class TrajectoryPlayback : public Motion {
public:
    TrajectoryPlayback(std::vector<double> times, std::vector<Joints> points, double speed_percent);
    bool step(double dt, Joints& q) override;
    bool paused = false;

private:
    std::vector<double> times_;
    std::vector<Joints> points_;
    double scale_, t_ = 0;
};

// Transparent transmission (pass-through streaming).
// Model: points are consumed one per period `t`; each reaches the output after the
// lookahead delay, then a short first-order filter smooths it. The real controller's
// smoothing algorithm is not documented; the delay is what matters for clients.
class PassThrough : public Motion {
public:
    PassThrough(const Joints& q_start, double period_s, double lookahead_s, double smoothness);
    bool step(double dt, Joints& q) override;   // runs until cleared
    void push(const Joints& q) { buffer_.push_back(q); }
    std::size_t buffered() const { return buffer_.size() + delay_.size(); }

private:
    double period_, lookahead_, tau_;
    std::deque<Joints> buffer_;   // received, waiting for their tick
    std::deque<Joints> delay_;    // consumed, inside the lookahead window
    Joints goal_, out_;
    double acc_ = 0;
};

// moveBySpeedj: constant joint velocity for a duration, with acceleration limits
class SpeedJ : public Motion {
public:
    SpeedJ(const Joints& v_target_deg_s, double acc_deg_s2, double duration_s);
    bool step(double dt, Joints& q) override;
    void stop(double acc_deg_s2);   // decelerate to zero, then finish

private:
    Joints v_{}, v_target_;
    double acc_, remaining_;
    bool stopping_ = false;
};

}  // namespace emu
