#include "camera/decoder.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libswscale/swscale.h>
}

namespace camera {

namespace {
std::string av_text(int err) {
    char text[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(err, text, sizeof text);
    return text;
}
}  // namespace

Decoder::~Decoder() { close(); }

void Decoder::close() {
    sws_freeContext(sws_);
    sws_ = nullptr;
    sws_w_ = sws_h_ = 0;
    sws_fmt_ = -1;
    av_packet_free(&packet_);
    av_frame_free(&frame_);
    avcodec_free_context(&ctx_);
}

std::error_code Decoder::fail(CameraErrc e, std::string detail, int av_error) {
    detail_ = av_error ? detail + ": " + av_text(av_error) : std::move(detail);
    return e;
}

std::error_code Decoder::open() {
    close();
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) return fail(CameraErrc::codec_not_found, "FFmpeg has no H.264 decoder");
    ctx_ = avcodec_alloc_context3(codec);
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (!ctx_ || !frame_ || !packet_) {
        close();
        return fail(CameraErrc::open_failed, "out of memory");
    }
    ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;   // output each frame as soon as it is decoded
    ctx_->thread_count = 1;                   // frame threading would hold frames back
    if (int err = avcodec_open2(ctx_, codec, nullptr); err < 0) {
        close();
        return fail(CameraErrc::open_failed, "avcodec_open2(h264)", err);
    }
    detail_.clear();
    return {};
}

std::error_code Decoder::decode(std::span<const std::uint8_t> access_unit, DecodedFrame& out, bool& got) {
    got = false;
    if (!ctx_) return fail(CameraErrc::bad_input, "decoder not open");
    // the packet points at the caller's bytes: decoded before this function returns
    packet_->data = const_cast<std::uint8_t*>(access_unit.data());
    packet_->size = static_cast<int>(access_unit.size());
    const int sent = avcodec_send_packet(ctx_, packet_);
    packet_->data = nullptr;
    packet_->size = 0;
    if (sent < 0 && sent != AVERROR(EAGAIN)) return fail(CameraErrc::decode_failed, "decode", sent);

    for (;;) {
        const int err = avcodec_receive_frame(ctx_, frame_);
        if (err == AVERROR(EAGAIN) || err == AVERROR_EOF) break;
        if (err < 0) return fail(CameraErrc::decode_failed, "decode", err);

        if (!sws_ || frame_->width != sws_w_ || frame_->height != sws_h_ || frame_->format != sws_fmt_) {
            sws_freeContext(sws_);
            sws_ = sws_getContext(frame_->width, frame_->height, static_cast<AVPixelFormat>(frame_->format),
                                  frame_->width, frame_->height, AV_PIX_FMT_RGBA, SWS_BILINEAR, nullptr, nullptr,
                                  nullptr);
            sws_w_ = frame_->width;
            sws_h_ = frame_->height;
            sws_fmt_ = frame_->format;
            if (!sws_) {
                av_frame_unref(frame_);
                return fail(CameraErrc::decode_failed, "no YUV to RGBA converter");
            }
        }
        out.width = frame_->width;
        out.height = frame_->height;
        out.rgba.resize(static_cast<std::size_t>(out.width) * out.height * 4);
        std::uint8_t* dst[1] = {out.rgba.data()};
        const int dst_stride[1] = {out.width * 4};
        sws_scale(sws_, frame_->data, frame_->linesize, 0, frame_->height, dst, dst_stride);
        av_frame_unref(frame_);
        got = true;   // with low delay there is one; if more came, the newest wins
    }
    return {};
}

}  // namespace camera
