// Monitor packet for port 8056, laid out as in the EC SDK-Socket Manual Ver3.19.2,
// table 3.1: big-endian, 1024 bytes, ending with the 0xEC8056EC matching word.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace emu {

struct StatusFields {
    std::uint64_t timestamp_ms = 0;
    std::array<double, 6> joint_pos{};        // deg
    std::array<double, 6> tcp_pose{};         // mm, rad (active tool)
    std::array<double, 6> flange_pose{};      // mm, rad
    std::array<double, 6> torque_permille{};
    int robot_state = 0;                      // 0 stop 1 pause 2 e-stop 3 running 4 alarm 5 collision
    int servo_ready = 0;
    int motor_synced = 0;
    int robot_mode = 2;                       // 0 teach 1 play 2 remote
    int collision = 0;
    int estop = 0;
    double tcp_speed = 0;                     // mm/s
    std::array<double, 6> joint_speed{};      // deg/s
    std::array<double, 6> joint_acc{};        // deg/s^2
    std::array<double, 6> joint_temperature{};
    std::array<double, 6> joint_torque{};     // Nm
};

constexpr std::size_t kPacketSize = 1024;
constexpr std::uint32_t kMatchingWord = 0xEC8056ECu;

std::vector<std::uint8_t> encode_status_packet(const StatusFields& f);

}  // namespace emu
