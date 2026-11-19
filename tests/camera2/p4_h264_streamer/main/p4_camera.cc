#include "p4_camera.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>

#include <cinttypes>
#include <span>

#include "esp_err.h"
extern "C" {
#include "esp_h264_alloc.h"   // header without extern "C"
}
#include "esp_h264_enc_param.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"

#include "p4_stamp.h"

namespace p4cam {

namespace {

constexpr const char* TAG = "camera";

// Waveshare ESP32-P4-ETH camera connector (same as Espressif's Function-EV-Board v1.5)
constexpr int kSccbPort = 0;
constexpr gpio_num_t kSccbScl = GPIO_NUM_8;
constexpr gpio_num_t kSccbSda = GPIO_NUM_7;
constexpr std::uint32_t kSccbHz = 100'000;

std::int64_t unix_ns_now() {
    timeval tv{};
    gettimeofday(&tv, nullptr);   // SNTP-disciplined once synchronised (see main.cc)
    return std::int64_t{tv.tv_sec} * 1'000'000'000 + std::int64_t{tv.tv_usec} * 1000;
}

}  // namespace

int Camera::open(const Config& config) {
    // --- sensor + MIPI-CSI + ISP (esp_video). Must run soon after reset (sensor power-up).
    static esp_video_init_csi_config_t csi{};
    csi.sccb_config.init_sccb = true;
    csi.sccb_config.i2c_config.port = kSccbPort;
    csi.sccb_config.i2c_config.scl_pin = kSccbScl;
    csi.sccb_config.i2c_config.sda_pin = kSccbSda;
    csi.sccb_config.freq = kSccbHz;
    csi.reset_pin = GPIO_NUM_NC;
    csi.pwdn_pin = GPIO_NUM_NC;
    static esp_video_init_config_t video{};
    video.csi = &csi;
    if (esp_err_t err = esp_video_init(&video); err != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init: %s (camera connected? ribbon the right way round?)", esp_err_to_name(err));
        return err;
    }

    fd_ = ::open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (fd_ < 0) {
        ESP_LOGE(TAG, "cannot open %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        return ESP_ERR_NOT_FOUND;
    }

    // ISP output: YUV420 in Espressif's packed layout, which is what the H.264 block reads
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_G_FMT, &format) != 0) return ESP_FAIL;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    if (ioctl(fd_, VIDIOC_S_FMT, &format) != 0) {
        ESP_LOGE(TAG, "the camera pipeline refused YUV420 (ISP pipeline enabled in menuconfig?)");
        return ESP_FAIL;
    }
    width_ = format.fmt.pix.width;
    height_ = format.fmt.pix.height;

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_G_PARM, &parm) == 0 && parm.parm.capture.timeperframe.numerator != 0)
        fps_ = parm.parm.capture.timeperframe.denominator / parm.parm.capture.timeperframe.numerator;
    if (fps_ == 0) fps_ = 30;

    v4l2_requestbuffers req{};
    req.count = kBufferCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) != 0) return ESP_FAIL;
    for (int i = 0; i < kBufferCount; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) != 0) return ESP_FAIL;
        void* p = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (p == MAP_FAILED) return ESP_ERR_NO_MEM;
        buffer_[i] = static_cast<std::uint8_t*>(p);
        buffer_size_ = buf.length;
        if (ioctl(fd_, VIDIOC_QBUF, &buf) != 0) return ESP_FAIL;
    }

    // --- hardware H.264 encoder: baseline, no B-frames, SPS/PPS before every IDR
    esp_h264_enc_cfg_hw_t cfg{};
    cfg.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;
    cfg.gop = config.gop;
    cfg.fps = static_cast<std::uint8_t>(fps_);
    cfg.res.width = static_cast<std::uint16_t>(width_);
    cfg.res.height = static_cast<std::uint16_t>(height_);
    cfg.rc.bitrate = config.bitrate_bps;
    cfg.rc.qp_min = config.qp_min;
    cfg.rc.qp_max = config.qp_max;
    if (esp_h264_enc_hw_new(&cfg, &encoder_) != ESP_H264_ERR_OK || esp_h264_enc_open(encoder_) != ESP_H264_ERR_OK ||
        esp_h264_enc_hw_get_param_hd(encoder_, &encoder_param_) != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 encoder setup failed (%" PRIu32 "x%" PRIu32 " @ %" PRIu32 " fps)", width_, height_, fps_);
        return ESP_FAIL;
    }
    // Worst case an IDR at low QP: a generous half of the raw YUV420 size
    h264_ = static_cast<std::uint8_t*>(
        esp_h264_aligned_calloc(128, 1, width_ * height_ * 3 / 4, &h264_capacity_, ESP_H264_MEM_SPIRAM));
    if (!h264_) return ESP_ERR_NO_MEM;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) != 0) return ESP_FAIL;

    ESP_LOGI(TAG, "%" PRIu32 "x%" PRIu32 " @ %" PRIu32 " fps, H.264 %" PRIu32 " kbit/s, gop %u, qp %u..%u", width_,
             height_, fps_, config.bitrate_bps / 1000, config.gop, config.qp_min, config.qp_max);
    return ESP_OK;
}

int Camera::encode_next(bool force_keyframe, camera::EncodedFrame& out) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_DQBUF, &buf) != 0) return ESP_FAIL;
    const std::int64_t capture_ns = unix_ns_now();   // frame fully read out: within one readout of capture
    const std::uint64_t frame_id = frame_id_++;      // +1 per camera frame: gaps = frames we skipped

    if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
        ioctl(fd_, VIDIOC_QBUF, &buf);
        return ESP_ERR_INVALID_STATE;
    }

    if (force_keyframe) esp_h264_enc_force_idr(&encoder_param_->base);
    esp_h264_enc_in_frame_t in{};
    in.raw_data.buffer = buffer_[buf.index];
    in.raw_data.len = buffer_size_;
    esp_h264_enc_out_frame_t enc{};
    enc.raw_data.buffer = h264_;
    enc.raw_data.len = h264_capacity_;
    const std::int64_t t0 = esp_timer_get_time();
    const esp_h264_err_t err = esp_h264_enc_process(encoder_, &in, &enc);
    encode_us_ += esp_timer_get_time() - t0;
    ioctl(fd_, VIDIOC_QBUF, &buf);   // camera buffer back to the driver as soon as it's encoded
    if (err != ESP_H264_ERR_OK || enc.length == 0) {
        ESP_LOGW(TAG, "encode failed (%d)", int(err));
        return ESP_FAIL;
    }

    stamp_access_unit(std::span<const std::uint8_t>(h264_, enc.length), camera::FrameStamp{frame_id, capture_ns, 0},
                      enc.frame_type == ESP_H264_FRAME_TYPE_IDR, out);
    return ESP_OK;
}

}  // namespace p4cam
