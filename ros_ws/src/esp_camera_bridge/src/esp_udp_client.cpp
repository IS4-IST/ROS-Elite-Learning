#include "esp_camera_bridge/esp_udp_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace esp_camera_bridge {

namespace {

// Little-endian read, independent of the host byte order
template <typename T>
T readLe(const uint8_t * p)
{
    T value = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
        value |= static_cast<T>(p[i]) << (8 * i);
    }
    return value;
}

}  // namespace

EspUdpClient::EspUdpClient(std::string esp_ip, uint16_t command_port, uint16_t udp_port)
: esp_ip_(std::move(esp_ip)), command_port_(command_port), udp_port_(udp_port)
{
}

EspUdpClient::~EspUdpClient()
{
    stop();
}

void EspUdpClient::start(FrameCallback on_frame)
{
    on_frame_ = std::move(on_frame);

    udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd_ < 0) {
        throw std::runtime_error("cannot create UDP socket: " + std::string(std::strerror(errno)));
    }

    int rcvbuf = 4 * 1024 * 1024;
    ::setsockopt(udp_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    // Lets receiveLoop wake up regularly and notice stop()
    timeval timeout{0, 500000};
    ::setsockopt(udp_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(udp_port_);
    if (::bind(udp_fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
        const std::string reason = std::strerror(errno);
        closeSockets();
        throw std::runtime_error("cannot bind UDP port " + std::to_string(udp_port_) + ": " + reason);
    }

    tcp_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_fd_ < 0) {
        const std::string reason = std::strerror(errno);
        closeSockets();
        throw std::runtime_error("cannot create TCP socket: " + reason);
    }

    sockaddr_in esp{};
    esp.sin_family = AF_INET;
    esp.sin_port = htons(command_port_);
    if (::inet_pton(AF_INET, esp_ip_.c_str(), &esp.sin_addr) != 1) {
        closeSockets();
        throw std::runtime_error("invalid ESP IP address: " + esp_ip_);
    }
    if (::connect(tcp_fd_, reinterpret_cast<sockaddr *>(&esp), sizeof(esp)) < 0) {
        const std::string reason = std::strerror(errno);
        closeSockets();
        throw std::runtime_error("cannot connect to " + esp_ip_ + ":" + std::to_string(command_port_) + ": " + reason);
    }

    // The ESP sends frames to the address that opened the TCP connection, so this must come first
    int nodelay = 1;
    ::setsockopt(tcp_fd_, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    running_ = true;
    thread_ = std::thread(&EspUdpClient::receiveLoop, this);
}

void EspUdpClient::stop()
{
    if (!thread_.joinable()) {
        return;
    }

    try {
        sendCommand("stop");
    } catch (const std::runtime_error &) {
        // The ESP may already be gone; nothing else to clean up on the device side
    }

    running_ = false;
    thread_.join();
    closeSockets();
}

void EspUdpClient::sendCommand(const std::string & command)
{
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    if (tcp_fd_ < 0) {
        throw std::runtime_error("not connected to the ESP");
    }

    const auto length = static_cast<uint32_t>(command.size());
    std::array<uint8_t, sizeof(uint32_t)> prefix{};
    for (size_t i = 0; i < prefix.size(); ++i) {
        prefix[i] = static_cast<uint8_t>(length >> (8 * i));
    }

    std::vector<uint8_t> message(prefix.begin(), prefix.end());
    message.insert(message.end(), command.begin(), command.end());

    size_t sent = 0;
    while (sent < message.size()) {
        const ssize_t n = ::send(tcp_fd_, message.data() + sent, message.size() - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            throw std::runtime_error("failed to send command \"" + command + "\": " + std::strerror(errno));
        }
        sent += static_cast<size_t>(n);
    }
}

void EspUdpClient::receiveLoop()
{
    std::array<uint8_t, kHeaderSize + kUdpPayload + 64> buffer{};
    while (running_) {
        const ssize_t n = ::recv(udp_fd_, buffer.data(), buffer.size(), 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;  // timeout: check running_ again
            }
            break;
        }
        handlePacket(buffer.data(), static_cast<size_t>(n));
    }
}

void EspUdpClient::handlePacket(const uint8_t * packet, size_t length)
{
    if (length < kHeaderSize) {
        return;
    }

    const auto frame_id = readLe<uint32_t>(packet);
    const auto packet_index = readLe<uint16_t>(packet + 4);
    const auto packet_count = readLe<uint16_t>(packet + 6);
    const auto frame_size = readLe<uint32_t>(packet + 8);

    if (packet_count == 0 || packet_index >= packet_count || frame_size == 0 || frame_size > kMaxFrameSize) {
        return;
    }
    if (last_complete_ && frame_id <= *last_complete_) {
        return;  // late packet of a frame that was already delivered or dropped
    }

    const uint8_t * payload = packet + kHeaderSize;
    const size_t payload_length = length - kHeaderSize;

    auto it = frames_.find(frame_id);
    if (it == frames_.end()) {
        // Never let incomplete frames pile up if packets keep getting lost
        if (frames_.size() >= kMaxFramesInFlight) {
            frames_.erase(frames_.begin());
        }
        PendingFrame fresh;
        fresh.data.resize(frame_size);
        fresh.received.assign(packet_count, false);
        it = frames_.emplace(frame_id, std::move(fresh)).first;
    }

    PendingFrame & frame = it->second;
    if (frame.data.size() != frame_size || frame.received.size() != packet_count) {
        return;  // inconsistent with the first packet of this frame
    }

    const size_t offset = static_cast<size_t>(packet_index) * kUdpPayload;
    if (offset + payload_length > frame.data.size()) {
        return;
    }
    std::memcpy(frame.data.data() + offset, payload, payload_length);

    if (!frame.received[packet_index]) {
        frame.received[packet_index] = true;
        ++frame.received_count;
    }
    if (frame.received_count < packet_count) {
        return;
    }

    // Complete: deliver it and drop this frame's older incomplete siblings, they will never be shown
    std::vector<uint8_t> jpeg = std::move(frame.data);
    frames_.erase(frames_.begin(), frames_.upper_bound(frame_id));
    last_complete_ = frame_id;

    if (on_frame_) {
        on_frame_(std::move(jpeg));
    }
}

void EspUdpClient::closeSockets()
{
    if (udp_fd_ >= 0) {
        ::close(udp_fd_);
        udp_fd_ = -1;
    }
    std::lock_guard<std::mutex> lock(tcp_mutex_);
    if (tcp_fd_ >= 0) {
        ::close(tcp_fd_);
        tcp_fd_ = -1;
    }
}

}  // namespace esp_camera_bridge
