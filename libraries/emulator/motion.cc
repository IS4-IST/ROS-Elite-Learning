#include "emulator/motion.h"

#include <algorithm>
#include <cmath>

namespace emu {
namespace {

void mat_to_quat(const Mat3& R, double q[4]) {
    mjtNum mat[9];
    for (int i = 0; i < 9; ++i) mat[i] = R[i];
    mju_mat2Quat(q, mat);
}

void slerp(const double a[4], const double b_in[4], double s, double out[4]) {
    double b[4] = {b_in[0], b_in[1], b_in[2], b_in[3]};
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (dot < 0) {
        for (double& v : b) v = -v;
        dot = -dot;
    }
    if (dot > 0.9995) {
        for (int i = 0; i < 4; ++i) out[i] = a[i] + s * (b[i] - a[i]);
    } else {
        const double th = std::acos(dot), sn = std::sin(th);
        const double wa = std::sin((1 - s) * th) / sn, wb = std::sin(s * th) / sn;
        for (int i = 0; i < 4; ++i) out[i] = wa * a[i] + wb * b[i];
    }
    const double n = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
    for (int i = 0; i < 4; ++i) out[i] /= n;
}

}  // namespace

// ---------------------------------------------------------------------------
TrapezoidProfile::TrapezoidProfile(double distance, double v_max, double a_max)
    : D_(std::abs(distance)), v_(std::max(v_max, 1e-9)), a_(std::max(a_max, 1e-9)) {
    ta_ = v_ / a_;
    if (a_ * ta_ * ta_ > D_) {          // triangular profile: never reaches v_max
        ta_ = std::sqrt(D_ / a_);
        v_ = a_ * ta_;
        T_ = 2 * ta_;
    } else {
        T_ = 2 * ta_ + (D_ - a_ * ta_ * ta_) / v_;
    }
}

double TrapezoidProfile::s(double t) const {
    if (D_ < 1e-12) return 1.0;
    t = std::clamp(t, 0.0, T_);
    double x;
    if (t < ta_) x = 0.5 * a_ * t * t;
    else if (t < T_ - ta_) x = 0.5 * a_ * ta_ * ta_ + v_ * (t - ta_);
    else {
        const double tr = T_ - t;
        x = D_ - 0.5 * a_ * tr * tr;
    }
    return std::clamp(x / D_, 0.0, 1.0);
}

// ---------------------------------------------------------------------------
static double max_delta(const Joints& a, const Joints& b) {
    double m = 0;
    for (int i = 0; i < 6; ++i) m = std::max(m, std::abs(b[i] - a[i]));
    return m;
}

JointMove::JointMove(const Joints& q0, const Joints& q1, double v, double a)
    : q0_(q0), q1_(q1), profile_(max_delta(q0, q1), v, a) {}

bool JointMove::step(double dt, Joints& q) {
    t_ += dt;
    const double s = profile_.s(t_);
    for (int i = 0; i < 6; ++i) q[i] = q0_[i] + s * (q1_[i] - q0_[i]);
    return t_ < profile_.duration();
}

// ---------------------------------------------------------------------------
LineMove::LineMove(const Kinematics& kin, const Pose& tool, const Joints& q0, const Joints& q1, double speed)
    : kin_(kin), tool_(tool), q1_(q1) {
    const Pose a = kin.forward(q0, tool), b = kin.forward(q1, tool);
    for (int i = 0; i < 3; ++i) { p0_[i] = a[i]; p1_[i] = b[i]; }
    mat_to_quat(rpy_to_mat(a[3], a[4], a[5]), quat0_);
    mat_to_quat(rpy_to_mat(b[3], b[4], b[5]), quat1_);
    const double dist = std::sqrt(std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2) + std::pow(b[2] - a[2], 2));
    // pure rotations: use an equivalent distance so the move still takes time
    double dot = std::abs(quat0_[0] * quat1_[0] + quat0_[1] * quat1_[1] + quat0_[2] * quat1_[2] + quat0_[3] * quat1_[3]);
    const double angle_deg = 2 * std::acos(std::min(1.0, dot)) * 180.0 / M_PI;
    const double equiv = std::max(dist, angle_deg * 2.0);   // 2 mm per degree
    profile_.emplace(equiv, speed, 4.0 * speed);
}

bool LineMove::step(double dt, Joints& q) {
    t_ += dt;
    const double s = profile_->s(t_);
    Pose target{};
    for (int i = 0; i < 3; ++i) target[i] = p0_[i] + s * (p1_[i] - p0_[i]);
    double qt[4];
    slerp(quat0_, quat1_, s, qt);
    mjtNum mat[9], quat[4] = {qt[0], qt[1], qt[2], qt[3]};
    mju_quat2Mat(mat, quat);
    Mat3 R;
    for (int i = 0; i < 9; ++i) R[i] = mat[i];
    mat_to_rpy(R, target[3], target[4], target[5]);
    auto sol = kin_.inverse(target, tool_, q);     // nearest to the previous command
    if (!sol) {
        error = "line motion: no IK solution along the path";
        return false;
    }
    q = *sol;
    if (t_ >= profile_->duration()) {
        q = q1_;
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
TrajectoryPlayback::TrajectoryPlayback(std::vector<double> times, std::vector<Joints> points, double speed_percent)
    : times_(std::move(times)), points_(std::move(points)), scale_(std::max(speed_percent, 1.0) / 100.0) {}

bool TrajectoryPlayback::step(double dt, Joints& q) {
    if (paused) return true;
    t_ += dt * scale_;
    if (t_ >= times_.back()) {
        q = points_.back();
        return false;
    }
    const auto it = std::upper_bound(times_.begin(), times_.end(), t_);
    const std::size_t k = std::max<std::size_t>(1, it - times_.begin()) - 1;
    const double a = (t_ - times_[k]) / (times_[k + 1] - times_[k]);
    for (int i = 0; i < 6; ++i) q[i] = points_[k][i] + a * (points_[k + 1][i] - points_[k][i]);
    return true;
}

// ---------------------------------------------------------------------------
PassThrough::PassThrough(const Joints& q_start, double period_s, double lookahead_s, double smoothness)
    : period_(period_s), lookahead_(lookahead_s), goal_(q_start), out_(q_start) {
    tau_ = 0.005 + 0.03 * std::clamp(smoothness, 0.0, 1.0);   // filter time constant, s
}

bool PassThrough::step(double dt, Joints& q) {
    acc_ += dt;
    while (acc_ >= period_ && !buffer_.empty()) {
        delay_.push_back(buffer_.front());
        buffer_.pop_front();
        acc_ -= period_;
    }
    if (buffer_.empty()) acc_ = std::min(acc_, period_);
    // a point leaves the lookahead window once the window holds lookahead/period points
    const std::size_t window = std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(lookahead_ / period_)));
    while (delay_.size() > window) {
        goal_ = delay_.front();
        delay_.pop_front();
    }
    const double alpha = 1.0 - std::exp(-dt / tau_);
    for (int i = 0; i < 6; ++i) out_[i] += alpha * (goal_[i] - out_[i]);
    q = out_;
    return true;
}

// ---------------------------------------------------------------------------
SpeedJ::SpeedJ(const Joints& v_target, double acc, double duration)
    : v_target_(v_target), acc_(std::max(acc, 1.0)), remaining_(duration) {}

void SpeedJ::stop(double acc) {
    acc_ = std::max(acc, 1.0);
    stopping_ = true;
}

bool SpeedJ::step(double dt, Joints& q) {
    remaining_ -= dt;
    if (remaining_ <= 0) stopping_ = true;
    bool moving = false;
    for (int i = 0; i < 6; ++i) {
        const double goal = stopping_ ? 0.0 : v_target_[i];
        const double dv = std::clamp(goal - v_[i], -acc_ * dt, acc_ * dt);
        v_[i] += dv;
        q[i] += v_[i] * dt;
        moving |= std::abs(v_[i]) > 1e-6;
    }
    return !stopping_ || moving;
}

}  // namespace emu
