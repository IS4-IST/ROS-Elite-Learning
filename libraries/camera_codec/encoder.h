// H.264 encoder for camera images (FFmpeg's libavcodec, libx264 by default).
//
//   camera::Encoder enc;
//   if (auto ec = enc.open({.width = 640, .height = 480, .fps = 30})) { ... enc.error_detail() ... }
//   camera::EncodedFrame frame;
//   enc.encode(rgb, /*bottom_up=*/true, {frame_id, capture_ns}, /*force_keyframe=*/false, frame);
//   // frame.bytes: Annex-B access unit with the timestamp SEI (see stream_format.h)
//
// Settings match what the receiver expects from the ESP32-P4: baseline profile, no
// B-frames, one packet out per frame in (zero latency), SPS/PPS repeated on every IDR.
// Not thread-safe: one thread uses one encoder.
#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "camera/errors.h"
#include "camera/stream_format.h"

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace camera {

struct EncoderConfig {
    int width = 640;             // multiples of 16 suit both x264 and the P4 encoder
    int height = 480;
    int fps = 30;
    int gop = 15;                // a keyframe at least every gop frames (0.5 s at 30 fps)
    int bitrate_kbps = 2000;
    std::string codec = "libx264";   // or h264_nvenc, h264_vaapi, ... if the machine has them
};

struct EncodedFrame {
    std::vector<std::uint8_t> bytes;   // Annex-B: [SPS PPS] SEI(stamp) slice(s)
    bool keyframe = false;
    FrameStamp stamp;
};

class Encoder {
public:
    Encoder() = default;
    ~Encoder();
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;

    std::error_code open(const EncoderConfig& config);
    bool is_open() const { return ctx_ != nullptr; }
    const EncoderConfig& config() const { return config_; }

    // rgb: width * height * 3 bytes, RGB24. bottom_up: rows stored last row first (OpenGL).
    // force_keyframe: make this frame an IDR (a new client is waiting).
    std::error_code encode(std::span<const std::uint8_t> rgb, bool bottom_up, const FrameStamp& stamp,
                           bool force_keyframe, EncodedFrame& out);

    const std::string& error_detail() const { return detail_; }

private:
    std::error_code fail(CameraErrc e, std::string detail, int av_error = 0);
    void close();

    EncoderConfig config_;
    AVCodecContext* ctx_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVPacket* packet_ = nullptr;
    SwsContext* sws_ = nullptr;
    std::int64_t pts_ = 0;
    std::string detail_;
};

}  // namespace camera
