#include "camera/errors.h"

namespace camera {

namespace {

struct Category : std::error_category {
    const char* name() const noexcept override { return "ec-camera"; }
    std::string message(int c) const override {
        switch (static_cast<CameraErrc>(c)) {
            case CameraErrc::codec_not_found: return "H.264 encoder not available";
            case CameraErrc::open_failed: return "could not open the encoder";
            case CameraErrc::bad_input: return "bad image for the encoder";
            case CameraErrc::encode_failed: return "encoding failed";
            case CameraErrc::listen_failed: return "could not listen on the camera port";
            case CameraErrc::decode_failed: return "decoding failed";
        }
        return "unknown camera error";
    }
};

}  // namespace

const std::error_category& camera_category() {
    static const Category category;
    return category;
}

}  // namespace camera
