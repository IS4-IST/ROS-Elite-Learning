#include "camera_view.h"

#include <GL/gl.h>
#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>

namespace twin {

bool parse_camera_source(const std::string& text, int index, CameraSource& out) {
    const auto eq = text.find('=');
    const std::string address = eq == std::string::npos ? text : text.substr(eq + 1);
    const auto colon = address.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= address.size()) return false;
    const int port = std::atoi(address.c_str() + colon + 1);
    if (port <= 0 || port > 65535) return false;
    out.name = eq == std::string::npos ? "camera " + std::to_string(index + 1) : text.substr(0, eq);
    out.host = address.substr(0, colon);
    out.port = static_cast<unsigned short>(port);
    return true;
}

CameraView::CameraView(asio::io_context& io, CameraSource source, int index)
    : source_(std::move(source)), index_(index), client_(io, source_.host, source_.port) {
    client_.on_frame([this](const camera::DecodedFrame& image, const camera::FrameStamp&) { upload(image); });
    client_.start();
}

CameraView::~CameraView() {
    client_.close();
    if (texture_) glDeleteTextures(1, &texture_);
}

void CameraView::upload(const camera::DecodedFrame& image) {
    if (!texture_) {
        glGenTextures(1, &texture_);
        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);   // RGBA rows are always 4-byte aligned
    if (image.width != tex_width_ || image.height != tex_height_) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     image.rgba.data());
        tex_width_ = image.width;
        tex_height_ = image.height;
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image.width, image.height, GL_RGBA, GL_UNSIGNED_BYTE,
                        image.rgba.data());
    }
    glBindTexture(GL_TEXTURE_2D, 0);
}

void CameraView::draw() {
    const camera::ClientStats& s = client_.stats();
    // live: a frame within the last three frame intervals (at least half a second)
    const double patience = std::max(0.5, s.fps > 0 ? 3.0 / s.fps : 0.0);
    const bool live = s.connected && texture_ &&
                      std::chrono::duration<double>(std::chrono::steady_clock::now() - s.last_frame).count() < patience;

    // first appearance: stacked down the right side; afterwards wherever you put it
    ImGui::SetNextWindowPos({940.0f + 30.0f * (index_ / 2), 10.0f + 400.0f * (index_ % 2)}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({440, 390}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints({220, 160}, {10000, 10000});
    const std::string title = "Camera: " + source_.name + "###camera" + std::to_string(index_);
    ImGui::Begin(title.c_str(), nullptr, fit_ ? ImGuiWindowFlags_NoScrollbar : ImGuiWindowFlags_HorizontalScrollbar);

    // the numbers
    const ImVec4 green{0.5f, 0.9f, 0.5f, 1}, amber{1.0f, 0.8f, 0.3f, 1}, red{1.0f, 0.4f, 0.35f, 1};
    ImGui::TextColored(live ? green : (s.connected ? amber : red), "%s", live ? "live" : client_.status().c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("%s:%u", source_.host.c_str(), source_.port);
    if (s.width > 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("%dx%d", s.width, s.height);
    }
    ImGui::TextWrapped("%.1f fps   %.0f kbit/s   decode %.1f ms   latency %.0f ms", s.fps, s.kbps, s.decode_ms,
                       s.latency_ms);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("latency = arrival - capture time, so it includes any offset between the\n"
                          "camera's clock and this PC's (keep the camera synchronised, e.g. SNTP)");
    ImGui::Text("frames %llu  skipped %llu  errors %llu", static_cast<unsigned long long>(s.frames),
                static_cast<unsigned long long>(s.skipped), static_cast<unsigned long long>(s.errors));
    ImGui::SameLine();
    ImGui::Checkbox("fit", &fit_);

    // the image, keeping its aspect ratio
    if (!texture_) {
        ImGui::TextDisabled("no image yet");
    } else {
        const ImTextureID id = static_cast<ImTextureID>(static_cast<std::intptr_t>(texture_));
        ImVec2 size{static_cast<float>(tex_width_), static_cast<float>(tex_height_)};
        if (fit_) {
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            const float scale = std::max(0.01f, std::min(avail.x / size.x, avail.y / size.y));
            size = {size.x * scale, size.y * scale};
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (avail.x - size.x) / 2));   // centred
        }
        ImGui::Image(id, size);
        if (!live) {   // grey out a frozen image so nobody mistakes it for the present
            const ImVec2 min = ImGui::GetItemRectMin(), max = ImGui::GetItemRectMax();
            ImGui::GetWindowDrawList()->AddRectFilled(min, max, IM_COL32(0, 0, 0, 140));
            ImGui::GetWindowDrawList()->AddText({min.x + 8, min.y + 8}, IM_COL32(255, 200, 80, 255), "frozen: no stream");
        }
    }
    ImGui::End();
}

}  // namespace twin
