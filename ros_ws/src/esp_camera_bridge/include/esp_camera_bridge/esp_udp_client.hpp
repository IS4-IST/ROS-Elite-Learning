#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace esp_camera_bridge {

// Client for the ESP32-CAM broadcasterUDP firmware, without any ROS dependency.
//
// Commands go over TCP: 4-byte little-endian length + "stream" | "stop" | "snapshot".
// JPEG frames come back over UDP, split into packets of at most kUdpPayload bytes, each prefixed
// with a 12-byte little-endian header: uint32 frame_id, uint16 packet_index, uint16 packet_count,
// uint32 frame_size. Lost packets lose the frame; nothing is retransmitted.
class EspUdpClient {
public:
    using FrameCallback = std::function<void(std::vector<uint8_t> && jpeg)>;

    EspUdpClient(std::string esp_ip, uint16_t command_port, uint16_t udp_port);
    ~EspUdpClient();

    EspUdpClient(const EspUdpClient &) = delete;
    EspUdpClient & operator=(const EspUdpClient &) = delete;

    // Binds the UDP port, then connects TCP to the ESP and starts the receive thread.
    // The UDP port is bound first so no frame is missed. Throws std::runtime_error on failure.
    void start(FrameCallback on_frame);

    // Sends "stop" (best effort), stops the receive thread and closes both sockets.
    void stop();

    // Throws std::runtime_error if the command cannot be sent.
    void sendCommand(const std::string & command);

private:
    static constexpr size_t kHeaderSize = 12;
    static constexpr size_t kUdpPayload = 1400;
    static constexpr size_t kMaxFrameSize = 2 * 1024 * 1024;
    static constexpr size_t kMaxFramesInFlight = 8;

    struct PendingFrame {
        std::vector<uint8_t> data;
        std::vector<bool> received;
        uint16_t received_count = 0;
    };

    void receiveLoop();
    void handlePacket(const uint8_t * packet, size_t length);
    void closeSockets();

    std::string esp_ip_;
    uint16_t command_port_;
    uint16_t udp_port_;

    int udp_fd_ = -1;
    int tcp_fd_ = -1;
    std::mutex tcp_mutex_;

    FrameCallback on_frame_;
    std::thread thread_;
    std::atomic<bool> running_{false};

    // Only touched from the receive thread
    std::map<uint32_t, PendingFrame> frames_;
    std::optional<uint32_t> last_complete_;
};

}  // namespace esp_camera_bridge
