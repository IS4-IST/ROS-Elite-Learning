#include "camera/encoder.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
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

Encoder::~Encoder() { close(); }

void Encoder::close() {
    sws_freeContext(sws_);
    sws_ = nullptr;
    av_packet_free(&packet_);
    av_frame_free(&frame_);
    avcodec_free_context(&ctx_);
}

std::error_code Encoder::fail(CameraErrc e, std::string detail, int av_error) {
    detail_ = av_error ? detail + ": " + av_text(av_error) : std::move(detail);
    return e;
}

std::error_code Encoder::open(const EncoderConfig& config) {
    close();
    config_ = config;
    pts_ = 0;
    if (config.width <= 0 || config.height <= 0 || config.width % 2 || config.height % 2)
        return fail(CameraErrc::bad_input, "width and height must be even and positive");

    const AVCodec* codec = avcodec_find_encoder_by_name(config.codec.c_str());
    if (!codec) return fail(CameraErrc::codec_not_found, "FFmpeg has no encoder named " + config.codec);

    ctx_ = avcodec_alloc_context3(codec);
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (!ctx_ || !frame_ || !packet_) {
        close();
        return fail(CameraErrc::open_failed, "out of memory");
    }
    ctx_->width = config.width;
    ctx_->height = config.height;
    ctx_->time_base = AVRational{1, config.fps};
    ctx_->framerate = AVRational{config.fps, 1};
    ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
    ctx_->gop_size = config.gop;
    ctx_->max_b_frames = 0;   // frames leave in capture order, no reordering delay
    ctx_->bit_rate = static_cast<std::int64_t>(config.bitrate_kbps) * 1000;
    // no AV_CODEC_FLAG_GLOBAL_HEADER: SPS/PPS stay in the stream, where the receiver needs them

    av_opt_set(ctx_->priv_data, "profile", "baseline", 0);
    if (config.codec == "libx264") {
        av_opt_set(ctx_->priv_data, "preset", "ultrafast", 0);
        av_opt_set(ctx_->priv_data, "tune", "zerolatency", 0);   // no lookahead: one frame in, one out
        av_opt_set(ctx_->priv_data, "forced-idr", "1", 0);       // a forced keyframe is an IDR
        av_opt_set(ctx_->priv_data, "x264-params", "repeat-headers=1", 0);   // SPS/PPS on every IDR
    }
    if (int err = avcodec_open2(ctx_, codec, nullptr); err < 0) {
        close();
        return fail(CameraErrc::open_failed, "avcodec_open2(" + config.codec + ")", err);
    }

    frame_->format = AV_PIX_FMT_YUV420P;
    frame_->width = config.width;
    frame_->height = config.height;
    if (int err = av_frame_get_buffer(frame_, 0); err < 0) {
        close();
        return fail(CameraErrc::open_failed, "av_frame_get_buffer", err);
    }
    sws_ = sws_getContext(config.width, config.height, AV_PIX_FMT_RGB24, config.width, config.height,
                          AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws_) {
        close();
        return fail(CameraErrc::open_failed, "no RGB to YUV converter");
    }
    detail_.clear();
    return {};
}

std::error_code Encoder::encode(std::span<const std::uint8_t> rgb, bool bottom_up, const FrameStamp& stamp,
                                bool force_keyframe, EncodedFrame& out) {
    if (!ctx_) return fail(CameraErrc::bad_input, "encoder not open");
    const int stride = config_.width * 3;
    if (rgb.size() != static_cast<std::size_t>(stride) * config_.height)
        return fail(CameraErrc::bad_input, "image is not " + std::to_string(config_.width) + "x" +
                                               std::to_string(config_.height) + " RGB");

    if (int err = av_frame_make_writable(frame_); err < 0) return fail(CameraErrc::encode_failed, "frame", err);
    // OpenGL rows come bottom-up: start at the last row and walk backwards (negative stride)
    const std::uint8_t* src[1] = {bottom_up ? rgb.data() + static_cast<std::size_t>(stride) * (config_.height - 1)
                                            : rgb.data()};
    const int src_stride[1] = {bottom_up ? -stride : stride};
    sws_scale(sws_, src, src_stride, 0, config_.height, frame_->data, frame_->linesize);

    frame_->pts = pts_++;
    frame_->pict_type = force_keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
    if (int err = avcodec_send_frame(ctx_, frame_); err < 0) return fail(CameraErrc::encode_failed, "send", err);

    out.bytes.clear();
    out.keyframe = false;
    out.stamp = stamp;
    for (;;) {
        const int err = avcodec_receive_packet(ctx_, packet_);
        if (err == AVERROR(EAGAIN) || err == AVERROR_EOF) break;
        if (err < 0) return fail(CameraErrc::encode_failed, "receive", err);

        // copy the packet, putting the timestamp SEI just before the first slice
        const std::span<const std::uint8_t> data(packet_->data, static_cast<std::size_t>(packet_->size));
        bool stamped = false;
        std::size_t copied = 0;
        for (const Nal& nal : split_nals(data)) {
            if (stamped || (nal.type() != kNalSlice && nal.type() != kNalIdr)) continue;
            std::size_t sc = static_cast<std::size_t>(nal.bytes.data() - data.data()) - 3;   // its start code
            if (sc > 0 && data[sc - 1] == 0) --sc;                                         // 4-byte form
            out.bytes.insert(out.bytes.end(), data.begin() + copied, data.begin() + sc);
            FrameStamp with_size = stamp;
            with_size.follow = static_cast<std::uint32_t>(data.size() - sc);   // the slices, to the end
            append_stamp_sei(out.bytes, with_size);
            copied = sc;
            stamped = true;
        }
        out.bytes.insert(out.bytes.end(), data.begin() + copied, data.end());
        out.keyframe = out.keyframe || (packet_->flags & AV_PKT_FLAG_KEY);
        av_packet_unref(packet_);
    }
    return {};
}

}  // namespace camera
