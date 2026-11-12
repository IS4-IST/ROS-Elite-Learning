// Error codes of the camera library (category "ec-camera"). Expected failures are
// std::error_code; the human-readable reason is in the object's error_detail().
#pragma once

#include <string>
#include <system_error>

namespace camera {

enum class CameraErrc {
    codec_not_found = 1,   // the requested encoder isn't in this FFmpeg build
    open_failed,           // the encoder refused the settings
    bad_input,             // wrong image size, or not opened
    encode_failed,         // FFmpeg failed on a frame
    listen_failed,         // could not open the TCP port
    decode_failed,         // FFmpeg failed on a received frame
};

const std::error_category& camera_category();

inline std::error_code make_error_code(CameraErrc e) { return {static_cast<int>(e), camera_category()}; }

}  // namespace camera

template <>
struct std::is_error_code_enum<camera::CameraErrc> : std::true_type {};
