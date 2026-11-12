// The camera stream format, shared by the simulator, the receiver in the twin and the
// ESP32-P4 firmware.
//
// A camera serves plain H.264 Annex-B over TCP: NAL units separated by start codes
// (00 00 00 01), nothing else on the wire. So `nc <ip> <port> | ffplay -f h264 -` plays it.
//
// Each frame (access unit) carries one SEI "user data unregistered" NAL unit (type 6,
// payload type 5) just before its first slice, with:
//
//   uuid       16 bytes   kTimestampUuid
//   version     1 byte    1
//   frame_id    8 bytes   big endian, +1 per encoded frame (a gap = frames dropped by the camera)
//   capture_ns  8 bytes   big endian, capture time in ns since the Unix epoch (the camera's
//                         clock: keep it synchronised with the PC, e.g. SNTP)
//   follow      4 bytes   big endian, how many bytes of this frame come after the SEI unit
//                         (its slices, start codes included)
//
// `follow` lets a receiver decode as soon as the frame's last byte arrives. Without it a
// frame only ends where the next one starts, which costs a whole frame of latency.
//
// Decoders ignore SEI units they don't know, so this costs ffplay nothing.
// Encoder settings the receiver relies on: baseline profile (no B-frames, so frames come
// out in capture order) and SPS + PPS repeated before every IDR, so a client can start at
// any keyframe.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace camera {

inline constexpr std::array<std::uint8_t, 16> kTimestampUuid = {
    0x45, 0x43, 0x36, 0x36, 0x2d, 0x63, 0x61, 0x6d,   // "EC66-cam"
    0x2d, 0x74, 0x73, 0x00, 0x8a, 0x1f, 0x4c, 0x01};  // "-ts" + random tail
inline constexpr std::uint8_t kTimestampVersion = 1;

struct FrameStamp {
    std::uint64_t frame_id = 0;
    std::int64_t capture_ns = 0;   // ns since the Unix epoch
    std::uint32_t follow = 0;      // bytes of the frame after the SEI unit
};

struct Nal {
    std::span<const std::uint8_t> bytes;   // from the NAL header on, without the start code
    int type() const { return bytes.empty() ? -1 : bytes[0] & 0x1f; }
};

// NAL types used here
inline constexpr int kNalSlice = 1, kNalIdr = 5, kNalSei = 6, kNalSps = 7, kNalPps = 8;

// Splits an Annex-B buffer into NAL units. The last unit runs to the end of `data`.
std::vector<Nal> split_nals(std::span<const std::uint8_t> data);

// Appends the timestamp SEI NAL unit (with start code) to `out`.
void append_stamp_sei(std::vector<std::uint8_t>& out, const FrameStamp& stamp);

// Reads a timestamp SEI unit. nullopt if `nal` is another kind of NAL or SEI.
std::optional<FrameStamp> parse_stamp_sei(const Nal& nal);

// Finds where a NAL unit that is complete in `data` ends: the start of the next start code
// at or after `from`, or npos. A stream reader uses it to cut units as bytes arrive.
inline constexpr std::size_t npos = static_cast<std::size_t>(-1);
std::size_t find_start_code(std::span<const std::uint8_t> data, std::size_t from = 0);

}  // namespace camera
