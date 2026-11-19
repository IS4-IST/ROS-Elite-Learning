// Puts the timestamp SEI into one encoded access unit (libraries/camera/stream_format.h).
// Kept apart from the camera code so it can be tested on a PC.
#pragma once

#include <cstdint>
#include <span>

#include "camera/encoder.h"   // camera::EncodedFrame

namespace p4cam {

// Copies `access_unit` (Annex-B, [SPS PPS] slice(s)) into `out` with the stamp SEI just before
// the first slice, its `follow` set to the bytes from that slice's start code to the end.
void stamp_access_unit(std::span<const std::uint8_t> access_unit, const camera::FrameStamp& stamp, bool keyframe,
                       camera::EncodedFrame& out);

}  // namespace p4cam
