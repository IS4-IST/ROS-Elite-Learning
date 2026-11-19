#include "p4_stamp.h"

namespace p4cam {

void stamp_access_unit(std::span<const std::uint8_t> data, const camera::FrameStamp& stamp, bool keyframe,
                       camera::EncodedFrame& out) {
    out.bytes.clear();
    out.bytes.reserve(data.size() + 64);
    out.keyframe = keyframe;
    out.stamp = stamp;
    std::size_t copied = 0;
    for (const camera::Nal& nal : camera::split_nals(data)) {
        if (nal.type() != camera::kNalSlice && nal.type() != camera::kNalIdr) continue;
        std::size_t sc = static_cast<std::size_t>(nal.bytes.data() - data.data()) - 3;   // its start code
        if (sc > 0 && data[sc - 1] == 0) --sc;                                         // 4-byte form
        out.bytes.insert(out.bytes.end(), data.begin(), data.begin() + sc);           // SPS, PPS, ...
        camera::FrameStamp with_size = stamp;
        with_size.follow = static_cast<std::uint32_t>(data.size() - sc);              // the slices, to the end
        camera::append_stamp_sei(out.bytes, with_size);
        copied = sc;
        break;
    }
    out.bytes.insert(out.bytes.end(), data.begin() + copied, data.end());
}

}  // namespace p4cam
