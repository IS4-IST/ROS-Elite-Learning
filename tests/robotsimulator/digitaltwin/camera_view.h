// One camera in the twin's GUI: receives its H.264 stream (simulator or ESP32-P4), keeps
// the latest frame in an OpenGL texture and draws it in a resizable ImGui window with the
// stream's numbers (fps, bitrate, decode time, latency, skipped frames).
//
// Everything runs on the GUI thread: the client is on the GUI's io_context, so a decoded
// frame goes straight into the texture, which needs the window's OpenGL context anyway.
// Destroy it before the io_context and before the window (declare it after both).
#pragma once

#include <string>

#include "camera/client.h"

namespace twin {

struct CameraSource {
    std::string name;   // window title
    std::string host;
    unsigned short port = 0;
};

// "NAME=HOST:PORT" or "HOST:PORT". False if it can't be read.
bool parse_camera_source(const std::string& text, int index, CameraSource& out);

class CameraView {
public:
    CameraView(asio::io_context& io, CameraSource source, int index);
    ~CameraView();
    CameraView(const CameraView&) = delete;
    CameraView& operator=(const CameraView&) = delete;

    void draw();   // between viewer.begin_frame() and end_frame()

private:
    void upload(const camera::DecodedFrame& image);

    CameraSource source_;
    int index_;
    camera::Client client_;
    unsigned int texture_ = 0;
    int tex_width_ = 0, tex_height_ = 0;
    bool fit_ = true;   // scale the image to the window; off: 1:1 with scroll bars
};

}  // namespace twin
