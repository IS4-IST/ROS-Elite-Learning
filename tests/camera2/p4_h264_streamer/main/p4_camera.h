// Camera capture and H.264 encoding on the ESP32-P4 (Waveshare ESP32-P4-ETH + OV5647).
//
//   CSI camera -> ISP -> YUV420 (V4L2, esp_video) -> hardware H.264 encoder (esp_h264)
//
// encode_next() produces one access unit in the repo's stream format
// (libraries/camera/stream_format.h): [SPS PPS] SEI(frame_id, capture_ns, follow) slice(s).
#pragma once

#include <atomic>
#include <cstdint>
#include <system_error>

#include "camera/encoder.h"   // camera::EncodedFrame (header only: no FFmpeg needed)

#include "esp_h264_enc_single_hw.h"

namespace p4cam {

struct Config {
    std::uint32_t bitrate_bps = 4'000'000;
    std::uint8_t gop = 30;        // a keyframe at least every gop frames
    std::uint8_t qp_min = 25;
    std::uint8_t qp_max = 40;
};

class Camera {
public:
    // Powers the sensor, starts capture and opens the encoder. esp_err_t-style codes.
    int open(const Config& config);

    // Blocks until the next camera frame, encodes it into `out`.
    // force_keyframe: make this frame an IDR (a client is waiting for one).
    int encode_next(bool force_keyframe, camera::EncodedFrame& out);

    std::uint32_t width() const { return width_; }
    std::uint32_t height() const { return height_; }
    std::uint32_t fps() const { return fps_; }
    std::int64_t encode_us_total() const { return encode_us_.load(); }

private:
    static constexpr int kBufferCount = 2;

    int fd_ = -1;
    std::uint32_t width_ = 0, height_ = 0, fps_ = 0;
    std::uint8_t* buffer_[kBufferCount] = {};
    std::uint32_t buffer_size_ = 0;

    esp_h264_enc_handle_t encoder_ = nullptr;
    esp_h264_enc_param_hw_handle_t encoder_param_ = nullptr;
    std::uint8_t* h264_ = nullptr;          // encoder output (DMA capable)
    std::uint32_t h264_capacity_ = 0;

    std::uint64_t frame_id_ = 0;
    std::atomic<std::int64_t> encode_us_{0};
};

}  // namespace p4cam
