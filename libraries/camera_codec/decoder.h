// H.264 decoder for camera frames (FFmpeg's libavcodec), low-delay: one frame in, one out.
//
//   camera::Decoder dec;
//   dec.open();
//   camera::DecodedFrame image;
//   bool got = false;
//   if (auto ec = dec.decode(access_unit, image, got)) { ... dec.error_detail() ... }
//   if (got) use(image.rgba, image.width, image.height);   // RGBA, rows top-down
//
// Feed it whole frames (access units), starting at a keyframe. Not thread-safe.
#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "camera/errors.h"

struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace camera {

struct DecodedFrame {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgba;   // width * height * 4, rows top-down
};

class Decoder {
public:
    Decoder() = default;
    ~Decoder();
    Decoder(const Decoder&) = delete;
    Decoder& operator=(const Decoder&) = delete;

    std::error_code open();
    std::error_code decode(std::span<const std::uint8_t> access_unit, DecodedFrame& out, bool& got);
    const std::string& error_detail() const { return detail_; }

private:
    std::error_code fail(CameraErrc e, std::string detail, int av_error = 0);
    void close();

    AVCodecContext* ctx_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVPacket* packet_ = nullptr;
    SwsContext* sws_ = nullptr;
    int sws_w_ = 0, sws_h_ = 0, sws_fmt_ = -1;
    std::string detail_;
};

}  // namespace camera
