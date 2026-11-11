// Connects to a camera stream (simulator or ESP32-P4) and reports once a second:
// frames, keyframes, bitrate, frames the camera dropped (gaps in frame_id) and latency
// (arrival time - capture time, so it includes clock offset between the two machines).
//
//   camera_probe [host] [port] [--seconds N] [--save stream.h264]
//
// --save writes the raw stream; play it with `ffplay stream.h264`.
#include <utility>   // before asio: Asio 1.18 uses std::exchange without including it
#include <asio.hpp>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "camera/stream_format.h"

namespace {

std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

struct Stats {
    int frames = 0, keyframes = 0, dropped = 0;
    std::size_t bytes = 0;
    double latency_sum_ms = 0, latency_max_ms = 0;
};

asio::awaitable<int> probe(std::string host, std::string port, int seconds, std::string save_path) {
    auto executor = co_await asio::this_coro::executor;
    asio::ip::tcp::resolver resolver(executor);
    asio::ip::tcp::socket socket(executor);
    std::error_code ec;
    auto endpoints = co_await resolver.async_resolve(host, port, asio::redirect_error(asio::use_awaitable, ec));
    if (!ec) co_await asio::async_connect(socket, endpoints, asio::redirect_error(asio::use_awaitable, ec));
    if (ec) {
        std::fprintf(stderr, "connect %s:%s: %s\n", host.c_str(), port.c_str(), ec.message().c_str());
        co_return 1;
    }
    std::printf("connected to %s:%s\n", host.c_str(), port.c_str());
    std::ofstream save;
    if (!save_path.empty()) save.open(save_path, std::ios::binary);

    std::vector<std::uint8_t> buffer;   // bytes not yet cut into complete NAL units
    std::uint8_t chunk[65536];
    Stats stats;
    bool have_last = false, saw_sps = false;
    std::uint64_t last_id = 0;
    const auto start = std::chrono::steady_clock::now();
    auto report = start + std::chrono::seconds(1);

    for (;;) {
        const std::size_t n = co_await socket.async_read_some(asio::buffer(chunk), asio::redirect_error(asio::use_awaitable, ec));
        if (ec) {
            std::printf("connection ended: %s\n", ec.message().c_str());
            co_return ec == asio::error::eof ? 0 : 1;
        }
        if (save) save.write(reinterpret_cast<const char*>(chunk), static_cast<std::streamsize>(n));
        buffer.insert(buffer.end(), chunk, chunk + n);
        stats.bytes += n;

        // every unit followed by another start code is complete
        const std::span<const std::uint8_t> data(buffer);
        std::size_t first = camera::find_start_code(data, 0);
        std::size_t consumed = 0;
        while (first != camera::npos) {
            const std::size_t next = camera::find_start_code(data, first + 3);
            if (next == camera::npos) break;
            std::size_t end = next;
            while (end > first + 3 && data[end - 1] == 0) --end;
            const camera::Nal nal{data.subspan(first + 3, end - first - 3)};
            if (nal.type() == camera::kNalSps) saw_sps = true;
            if (nal.type() == camera::kNalIdr) ++stats.keyframes;
            if (auto stamp = camera::parse_stamp_sei(nal)) {
                ++stats.frames;
                if (have_last && stamp->frame_id > last_id + 1) stats.dropped += static_cast<int>(stamp->frame_id - last_id - 1);
                last_id = stamp->frame_id;
                have_last = true;
                const double latency = (now_ns() - stamp->capture_ns) / 1e6;
                stats.latency_sum_ms += latency;
                stats.latency_max_ms = std::max(stats.latency_max_ms, latency);
            }
            consumed = next;
            first = next;
        }
        buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(consumed));

        const auto now = std::chrono::steady_clock::now();
        if (now >= report) {
            std::printf("%3d frames  %2d key  %6.0f kbit/s  skipped %d  latency avg %.1f max %.1f ms%s\n",
                        stats.frames, stats.keyframes, stats.bytes * 8 / 1000.0, stats.dropped,
                        stats.frames ? stats.latency_sum_ms / stats.frames : 0.0, stats.latency_max_ms,
                        saw_sps ? "" : "  (no SPS yet)");
            stats = Stats{};
            report += std::chrono::seconds(1);
            if (seconds > 0 && now - start >= std::chrono::seconds(seconds)) co_return 0;
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string host = "127.0.0.1", port = "5000", save;
    int seconds = 0, positional = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--seconds" && i + 1 < argc) seconds = std::stoi(argv[++i]);
        else if (a == "--save" && i + 1 < argc) save = argv[++i];
        else if (a[0] != '-' && positional == 0) { host = a; ++positional; }
        else if (a[0] != '-' && positional == 1) { port = a; ++positional; }
        else { std::fprintf(stderr, "usage: camera_probe [host] [port] [--seconds N] [--save file.h264]\n"); return 1; }
    }
    asio::io_context io;
    int result = 1;
    asio::co_spawn(io, probe(host, port, seconds, save), [&](std::exception_ptr, int r) { result = r; });
    io.run();
    return result;
}
