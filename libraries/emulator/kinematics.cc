#include "emulator/kinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace emu {
namespace {

constexpr double kDeg = M_PI / 180.0;

Mat3 mul(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) r[3 * i + j] += a[3 * i + k] * b[3 * k + j];
    return r;
}

Mat3 transpose(const Mat3& a) {
    return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

// rotation vector (axis * angle) of R
void rotvec(const Mat3& R, double w[3]) {
    const double c = std::max(-1.0, std::min(1.0, (R[0] + R[4] + R[8] - 1.0) / 2.0));
    const double angle = std::acos(c);
    const double v[3] = {R[7] - R[5], R[2] - R[6], R[3] - R[1]};
    const double s = std::sin(angle);
    if (s < 1e-9) {
        // angle ~ 0 (or ~pi, rare here): small-angle approximation
        for (int i = 0; i < 3; ++i) w[i] = 0.5 * v[i];
        return;
    }
    for (int i = 0; i < 3; ++i) w[i] = v[i] * angle / (2.0 * s);
}

}  // namespace

Mat3 rpy_to_mat(double rx, double ry, double rz) {
    const double cx = std::cos(rx), sx = std::sin(rx), cy = std::cos(ry), sy = std::sin(ry), cz = std::cos(rz),
                 sz = std::sin(rz);
    // Rz * Ry * Rx
    return {cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
            sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
            -sy,     cy * sx,                cy * cx};
}

void mat_to_rpy(const Mat3& R, double& rx, double& ry, double& rz) {
    ry = std::atan2(-R[6], std::sqrt(R[0] * R[0] + R[3] * R[3]));
    rz = std::atan2(R[3], R[0]);
    rx = std::atan2(R[7], R[8]);
}

Kinematics::Kinematics(const mjModel* m) : m_(m), d_(mj_makeData(m)) {
    site_ = mj_name2id(m, mjOBJ_SITE, "tcp");
    if (site_ < 0) throw std::runtime_error("model has no site named 'tcp'");
    if (m->nq < 6) throw std::runtime_error("model needs 6 joints");
}

Kinematics::~Kinematics() { mj_deleteData(d_); }

void Kinematics::site_pose(const Joints& q, double pos_m[3], Mat3& R) const {
    for (int i = 0; i < 6; ++i) d_->qpos[i] = q[i] * kDeg;
    mj_kinematics(m_, d_);
    for (int i = 0; i < 3; ++i) pos_m[i] = d_->site_xpos[3 * site_ + i];
    for (int i = 0; i < 9; ++i) R[i] = d_->site_xmat[9 * site_ + i];
}

Pose Kinematics::forward(const Joints& q, const Pose& tool) const {
    double p[3];
    Mat3 R;
    site_pose(q, p, R);
    const Mat3 Rt = rpy_to_mat(tool[3], tool[4], tool[5]);
    const Mat3 Rtcp = mul(R, Rt);
    Pose out{};
    for (int i = 0; i < 3; ++i) {
        // tool offset is in the flange frame, mm
        const double off = R[3 * i] * tool[0] + R[3 * i + 1] * tool[1] + R[3 * i + 2] * tool[2];
        out[i] = p[i] * 1000.0 + off;
    }
    mat_to_rpy(Rtcp, out[3], out[4], out[5]);
    return out;
}

bool Kinematics::within_limits(const Joints& q) const {
    for (int i = 0; i < 6; ++i) {
        if (m_->jnt_limited[i] &&
            (q[i] * kDeg < m_->jnt_range[2 * i] - 1e-9 || q[i] * kDeg > m_->jnt_range[2 * i + 1] + 1e-9))
            return false;
    }
    return true;
}

std::optional<Joints> Kinematics::inverse(const Pose& pose, const Pose& tool, const Joints& ref) const {
    // desired flange (site) pose from the desired TCP pose
    const Mat3 Rtcp = rpy_to_mat(pose[3], pose[4], pose[5]);
    const Mat3 Rt = rpy_to_mat(tool[3], tool[4], tool[5]);
    const Mat3 Rdes = mul(Rtcp, transpose(Rt));
    double pdes[3];
    for (int i = 0; i < 3; ++i) {
        const double off = Rdes[3 * i] * tool[0] + Rdes[3 * i + 1] * tool[1] + Rdes[3 * i + 2] * tool[2];
        pdes[i] = (pose[i] - off) / 1000.0;
    }

    Joints q = ref;
    const int nv = m_->nv;
    std::vector<mjtNum> jacp(3 * nv), jacr(3 * nv);
    for (int iter = 0; iter < 200; ++iter) {
        double p[3];
        Mat3 R;
        site_pose(q, p, R);
        // error: position (m) and orientation (rotation vector of Rdes * R^T, world frame)
        double e[6];
        for (int i = 0; i < 3; ++i) e[i] = pdes[i] - p[i];
        rotvec(mul(Rdes, transpose(R)), e + 3);
        double err_p = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        double err_r = std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);
        if (err_p < 1e-6 && err_r < 1e-5) return within_limits(q) ? std::optional<Joints>(q) : std::nullopt;

        mj_comPos(m_, d_);                 // needed by the Jacobian
        mj_jacSite(m_, d_, jacp.data(), jacr.data(), site_);
        // J (6 x 6, first 6 dofs), dq = J^T (J J^T + lambda^2 I)^-1 e
        double J[36];
        for (int c = 0; c < 6; ++c) {
            for (int r = 0; r < 3; ++r) {
                J[6 * r + c] = jacp[r * nv + c];
                J[6 * (r + 3) + c] = jacr[r * nv + c];
            }
        }
        double A[36];
        const double lambda2 = 1e-4;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                double s = 0;
                for (int k = 0; k < 6; ++k) s += J[6 * i + k] * J[6 * j + k];
                A[6 * i + j] = s + (i == j ? lambda2 : 0.0);
            }
        double y[6];
        if (mju_cholFactor(A, 6, 0) < 6) return std::nullopt;
        mju_cholSolve(y, A, e, 6);
        for (int c = 0; c < 6; ++c) {
            double dq = 0;
            for (int r = 0; r < 6; ++r) dq += J[6 * r + c] * y[r];
            dq = std::max(-0.2, std::min(0.2, dq));             // limit the step (rad)
            double qn = q[c] * kDeg + dq;
            if (m_->jnt_limited[c]) qn = std::max(m_->jnt_range[2 * c], std::min(m_->jnt_range[2 * c + 1], qn));
            q[c] = qn / kDeg;
        }
    }
    return std::nullopt;
}

}  // namespace emu
