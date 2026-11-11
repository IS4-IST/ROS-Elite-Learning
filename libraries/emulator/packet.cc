#include "emulator/packet.h"

#include <cstring>

namespace emu {
namespace {

class Writer {
public:
    explicit Writer(std::vector<std::uint8_t>& p) : p_(p) {}
    void u(std::uint64_t v, int bytes) {
        for (int i = bytes - 1; i >= 0; --i) p_[pos_++] = static_cast<std::uint8_t>(v >> (8 * i));
    }
    void i32(int v) { u(static_cast<std::uint32_t>(v), 4); }
    void d(double v) {
        std::uint64_t bits;
        std::memcpy(&bits, &v, 8);
        u(bits, 8);
    }
    void ds(const double* v, int n, int total) {          // n values, padded with zeros to `total`
        for (int i = 0; i < total; ++i) d(i < n ? v[i] : 0.0);
    }
    void zeros_d(int n) { for (int i = 0; i < n; ++i) d(0.0); }

private:
    std::vector<std::uint8_t>& p_;
    std::size_t pos_ = 0;
};

}  // namespace

std::vector<std::uint8_t> encode_status_packet(const StatusFields& f) {
    std::vector<std::uint8_t> p(kPacketSize, 0);
    Writer w(p);
    w.u(kPacketSize, 4);                          // MessageSize
    w.u(f.timestamp_ms, 8);                       // TimeStamp
    w.u(2, 1);                                    // autorun_cycleMode
    w.ds(f.joint_pos.data(), 6, 8);               // machinePos[8]
    w.ds(f.tcp_pose.data(), 6, 6);                // machinePose[6]
    w.ds(f.tcp_pose.data(), 6, 6);                // machineUserPose[6] (user frame = base)
    w.ds(f.torque_permille.data(), 6, 8);         // torque[8]
    w.i32(f.robot_state);                         // robotState
    w.i32(f.servo_ready);                         // servoReady
    w.i32(f.motor_synced);                        // can_motor_run
    for (int i = 0; i < 8; ++i) w.i32(0);         // motor_speed[8]
    w.i32(f.robot_mode);                          // robotMode
    w.zeros_d(3);                                 // analog_ioInput[3]
    w.zeros_d(5);                                 // analog_ioOutput[5]
    w.u(0, 8);                                    // digital_ioInput
    w.u(0, 8);                                    // digital_ioOutput
    w.u(static_cast<std::uint64_t>(f.collision), 1);
    w.ds(f.flange_pose.data(), 6, 6);             // machineFlangePose
    w.ds(f.flange_pose.data(), 6, 6);             // machineUserFlangePose
    w.u(static_cast<std::uint64_t>(f.estop), 1);  // emergencyStopState
    w.d(f.tcp_speed);                             // tcp_speed
    w.ds(f.joint_speed.data(), 6, 8);             // joint_speed[8]
    w.d(0.0);                                     // tcpacc
    w.ds(f.joint_acc.data(), 6, 8);               // jointacc[8]
    w.ds(f.joint_temperature.data(), 6, 6);       // joint temperature
    w.ds(f.joint_torque.data(), 6, 6);            // joint torque
    w.zeros_d(6);                                 // estimated external joint torque
    w.zeros_d(6);                                 // estimated external TCP force
    w.u(0, 1);                                    // drag state
    w.u(0, 1);                                    // force sensor connected
    // reserved bytes stay zero; matching word at the end
    for (int i = 0; i < 4; ++i) p[kPacketSize - 4 + i] = static_cast<std::uint8_t>(kMatchingWord >> (8 * (3 - i)));
    return p;
}

}  // namespace emu
