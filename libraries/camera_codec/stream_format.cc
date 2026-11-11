#include "camera/stream_format.h"

#include <algorithm>

namespace camera {

std::size_t find_start_code(std::span<const std::uint8_t> data, std::size_t from) {
    for (std::size_t i = from; i + 2 < data.size(); ++i) {
        if (data[i + 2] > 1) { i += 2; continue; }   // can't be inside 00 00 01: skip ahead
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) return i;
    }
    return npos;
}

std::vector<Nal> split_nals(std::span<const std::uint8_t> data) {
    std::vector<Nal> nals;
    std::size_t sc = find_start_code(data, 0);
    while (sc != npos) {
        const std::size_t begin = sc + 3;
        const std::size_t next = find_start_code(data, begin);
        std::size_t end = next == npos ? data.size() : next;
        while (end > begin && data[end - 1] == 0) --end;   // the leading 00 of a 4-byte start code
        if (end > begin) nals.push_back(Nal{data.subspan(begin, end - begin)});
        sc = next;
    }
    return nals;
}

namespace {

void put_be(std::uint8_t* p, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = static_cast<std::uint8_t>(v >> (56 - 8 * i));
}

std::uint64_t get_be(const std::uint8_t* p) {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
    return v;
}

// Inside a NAL unit, 00 00 followed by 00..03 would look like a start code: the encoder
// inserts 03 after the two zeros ("emulation prevention"), the reader removes it.
void escape(std::vector<std::uint8_t>& out, std::span<const std::uint8_t> rbsp) {
    int zeros = 0;
    for (std::uint8_t b : rbsp) {
        if (zeros >= 2 && b <= 3) {
            out.push_back(3);
            zeros = 0;
        }
        out.push_back(b);
        zeros = b == 0 ? zeros + 1 : 0;
    }
}

std::vector<std::uint8_t> unescape(std::span<const std::uint8_t> nal) {
    std::vector<std::uint8_t> rbsp;
    rbsp.reserve(nal.size());
    int zeros = 0;
    for (std::uint8_t b : nal) {
        if (zeros >= 2 && b == 3) {   // drop the inserted byte
            zeros = 0;
            continue;
        }
        rbsp.push_back(b);
        zeros = b == 0 ? zeros + 1 : 0;
    }
    return rbsp;
}

constexpr std::size_t kPayloadSize = 16 + 1 + 8 + 8 + 4;
constexpr int kSeiUserDataUnregistered = 5;

}  // namespace

void append_stamp_sei(std::vector<std::uint8_t>& out, const FrameStamp& stamp) {
    std::array<std::uint8_t, 3 + kPayloadSize + 1> rbsp{};
    std::size_t i = 0;
    rbsp[i++] = kNalSei;                                     // nal_ref_idc 0, type 6
    rbsp[i++] = kSeiUserDataUnregistered;                    // payload type (< 255: one byte)
    rbsp[i++] = static_cast<std::uint8_t>(kPayloadSize);     // payload size (< 255: one byte)
    for (std::uint8_t b : kTimestampUuid) rbsp[i++] = b;
    rbsp[i++] = kTimestampVersion;
    put_be(rbsp.data() + i, stamp.frame_id);
    i += 8;
    put_be(rbsp.data() + i, static_cast<std::uint64_t>(stamp.capture_ns));
    i += 8;
    for (int shift = 24; shift >= 0; shift -= 8) rbsp[i++] = static_cast<std::uint8_t>(stamp.follow >> shift);
    rbsp[i] = 0x80;                                          // rbsp trailing bits

    const std::uint8_t start[] = {0, 0, 0, 1};
    out.insert(out.end(), std::begin(start), std::end(start));
    escape(out, rbsp);
}

std::optional<FrameStamp> parse_stamp_sei(const Nal& nal) {
    if (nal.type() != kNalSei) return std::nullopt;
    const std::vector<std::uint8_t> rbsp = unescape(nal.bytes);
    // header, type, size, payload
    if (rbsp.size() < 3 + kPayloadSize || rbsp[1] != kSeiUserDataUnregistered || rbsp[2] != kPayloadSize)
        return std::nullopt;
    const std::uint8_t* p = rbsp.data() + 3;
    if (!std::equal(kTimestampUuid.begin(), kTimestampUuid.end(), p)) return std::nullopt;
    if (p[16] != kTimestampVersion) return std::nullopt;
    FrameStamp stamp;
    stamp.frame_id = get_be(p + 17);
    stamp.capture_ns = static_cast<std::int64_t>(get_be(p + 25));
    stamp.follow = (std::uint32_t{p[33]} << 24) | (std::uint32_t{p[34]} << 16) | (std::uint32_t{p[35]} << 8) | p[36];
    return stamp;
}

}  // namespace camera
