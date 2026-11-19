// PC check of the P4 firmware's stream path, without the board:
//   access units from an .h264 file  ->  p4cam::stamp_access_unit (the firmware's code)
//   ->  camera::Server (the firmware's server)  ->  TCP  ->  camera::Client (the twin's receiver)
// and checks that every frame arrives decoded, in order, with the right frame_id and size.
//
//   stream_check test.h264 [port]       (make test.h264 with the command in CMakeLists.txt)
//
// Exit code 0 = pass.
#include <utility>
#include <asio.hpp>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <vector>

#include "camera/client.h"
#include "camera/server.h"
#include "camera/stream_format.h"
#include "p4_stamp.h"

namespace {

std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// Cuts the file into access units (frames). A frame can have several slices (x264 uses one
// per thread); a new frame starts with a non-slice unit (SPS, PPS, SEI, AUD) after a slice, or
// with a slice whose first_mb_in_slice is 0 (first bit after the NAL header set: ue(v) = 0).
std::vector<std::vector<std::uint8_t>> split_access_units(const std::vector<std::uint8_t>& file, std::vector<bool>& idr) {
    std::vector<std::vector<std::uint8_t>> units;
    const std::span<const std::uint8_t> data(file);
    std::size_t begin = 0;
    bool in_slices = false, key = false;
    auto cut = [&](std::size_t at) {
        std::size_t end = at;
        if (end > begin && data[end - 1] == 0) --end;   // keep the 4-byte start code with the next unit
        units.emplace_back(data.begin() + begin, data.begin() + end);
        idr.push_back(key);
        begin = end;
        key = false;
    };
    for (std::size_t sc = camera::find_start_code(data, 0); sc != camera::npos;
         sc = camera::find_start_code(data, sc + 3)) {
        const int type = data[sc + 3] & 0x1f;
        const bool slice = type == camera::kNalSlice || type == camera::kNalIdr;
        const bool first_slice = slice && (data[sc + 4] & 0x80);
        if (in_slices && (!slice || first_slice)) cut(sc);
        in_slices = slice;
        key = key || type == camera::kNalIdr;
    }
    if (begin < data.size()) cut(data.size());
    return units;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s file.h264 [port]\n", argv[0]);
        return 2;
    }
    std::ifstream in(argv[1], std::ios::binary);
    const std::vector<std::uint8_t> file((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<bool> idr;
    const auto units = split_access_units(file, idr);
    const unsigned short port = argc > 2 ? static_cast<unsigned short>(std::atoi(argv[2])) : 5600;
    std::printf("%zu access units, %zu keyframes\n", units.size(), std::size_t(std::count(idr.begin(), idr.end(), true)));

    asio::io_context io;
    camera::Server server(io);
    if (auto ec = server.listen("127.0.0.1", port)) {
        std::fprintf(stderr, "listen: %s\n", server.error_detail().c_str());
        return 1;
    }

    // Frame ids as the firmware makes them, with a deliberate gap (camera frame 40 "not encoded")
    std::vector<std::uint64_t> sent_ids;
    std::uint64_t next_id = 0;

    std::vector<std::uint64_t> got_ids;
    int bad_size = 0;
    camera::Client client(io, "127.0.0.1", port);
    client.on_frame([&](const camera::DecodedFrame& img, const camera::FrameStamp& stamp) {
        got_ids.push_back(stamp.frame_id);
        if (img.width != 1280 || img.height != 960) bad_size++;
    });
    client.start();

    // Send at 30 fps after the client had time to connect; it starts at the first keyframe it sees
    asio::steady_timer timer(io);
    std::size_t index = 0;
    std::function<void()> tick = [&] {
        timer.expires_after(std::chrono::milliseconds(33));
        timer.async_wait([&](std::error_code) {
            if (index == units.size()) {
                timer.expires_after(std::chrono::milliseconds(300));   // let the last frames arrive
                timer.async_wait([&](std::error_code) { io.stop(); });
                return;
            }
            if (next_id == 40) next_id++;   // the gap
            camera::EncodedFrame frame;
            p4cam::stamp_access_unit(units[index], camera::FrameStamp{next_id, now_ns(), 0}, idr[index], frame);
            if (server.clients() > 0) sent_ids.push_back(next_id);
            server.broadcast(std::make_shared<const camera::EncodedFrame>(std::move(frame)));
            server.wants_keyframe();   // the file has fixed keyframes; nothing to force here
            next_id++;
            index++;
            tick();
        });
    };
    timer.expires_after(std::chrono::milliseconds(300));
    timer.async_wait([&](std::error_code) { tick(); });
    io.run();

    const camera::ClientStats& st = client.stats();
    std::printf("sent %zu while connected, decoded %zu, decode errors %llu, skipped (id gaps) %llu, bad size %d\n",
                sent_ids.size(), got_ids.size(), (unsigned long long)st.errors, (unsigned long long)st.skipped,
                bad_size);
    // The client starts at the first keyframe after connecting; from there on everything must arrive
    bool ordered = !got_ids.empty();
    for (std::size_t i = 1; i < got_ids.size(); ++i) ordered = ordered && got_ids[i] > got_ids[i - 1];
    const bool tail_matches =
        !got_ids.empty() && std::equal(got_ids.begin(), got_ids.end(), sent_ids.end() - got_ids.size());
    const bool pass = ordered && tail_matches && st.errors == 0 && bad_size == 0 && st.skipped == 1 &&
                      got_ids.size() + 30 >= sent_ids.size();
    std::printf("first decoded id %llu, last %llu -> %s\n", got_ids.empty() ? 0ull : (unsigned long long)got_ids.front(),
                got_ids.empty() ? 0ull : (unsigned long long)got_ids.back(), pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
