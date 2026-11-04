// esp32cam_viewer: receives the ESP32-CAM stream and shows it with OpenCV.
//
//   esp32cam_viewer <esp32-ip> [tcp_port=3333] [udp_port=3334]
//
// Keys in the window: q / Esc = quit, s = save the current frame as JPEG.
#include "frame_receiver.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std::chrono;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <esp32-ip> [tcp_port=3333] [udp_port=3334]\n", argv[0]);
        return 1;
    }
    std::string host = argv[1];
    uint16_t tcp_port = 3333, udp_port = 3334;
    if (argc > 2) tcp_port = uint16_t(std::stoi(argv[2]));
    if (argc > 3) udp_port = uint16_t(std::stoi(argv[3]));

    // Newest frame, handed from the network thread to the display thread
    std::mutex latest_mutex;
    std::optional<esp32cam::Frame> latest;

    asio::io_context io;
    esp32cam::FrameReceiver receiver(io, host, tcp_port, udp_port, [&](esp32cam::Frame&& f) {
        std::lock_guard<std::mutex> lock(latest_mutex);
        latest = std::move(f);                         // an older, unshown frame is simply replaced
    });

    std::atomic<bool> quit{false};
    asio::signal_set signals(io, SIGINT, SIGTERM);
    signals.async_wait([&](std::error_code, int) { quit = true; });

    auto work = asio::make_work_guard(io);
    std::thread net([&] { io.run(); });
    receiver.start();

    auto window_start = steady_clock::now();
    uint64_t window_frames = receiver.stats().completed;
    double fps = 0.0;

    cv::Mat image;                       // last decoded frame (kept on screen until a newer one arrives)
    std::vector<uint8_t> current_jpeg;   // its JPEG bytes, for saving
    bool redraw = false;                 // only redraw when there is something new to show
    //cv::namedWindow("ESP32-CAM", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ESP32-CAM", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("ESP32-CAM", 1920, 1080);

    while (!quit) {
        // fps over 2-second windows, plus a status line
        auto now = steady_clock::now();
        if (now - window_start >= seconds(2)) {
            auto s = receiver.stats();
            fps = (s.completed - window_frames) / duration<double>(now - window_start).count();
            window_start = now;
            window_frames = s.completed;
            redraw = true;
            std::fprintf(stderr, "%s | %.1f fps | frames %llu | dropped %llu | bad packets %llu\n",
                         receiver.connected() ? "connected" : "not connected", fps,
                         (unsigned long long)s.completed, (unsigned long long)s.dropped,
                         (unsigned long long)s.bad_packets);
        }

        std::optional<esp32cam::Frame> frame;
        {
            std::lock_guard<std::mutex> lock(latest_mutex);
            frame.swap(latest);
        }

        if (frame) {
            cv::Mat decoded = cv::imdecode(frame->jpeg, cv::IMREAD_COLOR);
            if (!decoded.empty()) {
                image = decoded;
                current_jpeg = std::move(frame->jpeg);
                redraw = true;
            }
        }
        if (redraw && !image.empty()) {
            redraw = false;
            cv::Mat view = image.clone();
            char text[96];
            std::snprintf(text, sizeof(text), "%s  %.1f fps",
                          receiver.connected() ? "connected" : "NOT CONNECTED", fps);
            cv::putText(view, text, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
            cv::imshow("ESP32-CAM", view);
        }

        int key = cv::waitKey(1) & 0xFF;     // also lets the window redraw; ~1 ms wait
        if (key == 'q' || key == 27) quit = true;
        if (key == 's' && !current_jpeg.empty()) {
            std::string name = "frame_" + std::to_string(
                duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()) + ".jpg";
            std::ofstream(name, std::ios::binary).write(
                reinterpret_cast<const char*>(current_jpeg.data()), std::streamsize(current_jpeg.size()));
            std::fprintf(stderr, "saved %s\n", name.c_str());
        }
        static bool was_visible = false;
        const double visible = cv::getWindowProperty("ESP32-CAM", cv::WND_PROP_VISIBLE);
        if(visible >= 1) was_visible = true;
        else if (was_visible) quit = true;
    }

    receiver.stop();
    work.reset();
    io.stop();
    net.join();
    return 0;
}
