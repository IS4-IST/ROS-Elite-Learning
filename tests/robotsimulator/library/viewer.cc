#include "viewer.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdio>
#include <stdexcept>

namespace ec {

Viewer::Viewer(const mjModel* m, const char* title, int width, int height) : m_(m) {
    if (!glfwInit()) throw std::runtime_error("glfwInit failed (is there a display?)");
    glfwWindowHint(GLFW_SAMPLES, 4);
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) throw std::runtime_error("could not open a window");
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);   // the rendering timer paces frames; vsync would block the io_context

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);
    mjv_makeScene(m_, &scn_, 4000);   // room for the ghost copy as well
    mjr_makeContext(m_, &con_, mjFONTSCALE_150);
    cam_.lookat[0] = 0.2; cam_.lookat[1] = 0.0; cam_.lookat[2] = 0.4;
    cam_.distance = 2.0; cam_.azimuth = 135; cam_.elevation = -20;

    // Our callbacks first: ImGui's GLFW backend chains to them after handling the event.
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, on_key);
    glfwSetMouseButtonCallback(window_, on_button);
    glfwSetCursorPosCallback(window_, on_move);
    glfwSetScrollCallback(window_, on_scroll);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;   // don't write imgui.ini next to the binary
    ImGui::StyleColorsDark();
    float xscale = 1, yscale = 1;
    glfwGetWindowContentScale(window_, &xscale, &yscale);
    if (xscale > 1) {
        ImGui::GetStyle().ScaleAllSizes(xscale);
        ImGui::GetIO().FontGlobalScale = xscale;
    }
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 130");
}

Viewer::~Viewer() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    mjr_freeContext(&con_);
    mjv_freeScene(&scn_);
    glfwDestroyWindow(window_);
    glfwTerminate();
}

bool Viewer::should_close() const { return glfwWindowShouldClose(window_); }
void Viewer::close() { glfwSetWindowShouldClose(window_, 1); }

void Viewer::begin_frame() {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void Viewer::update_scene(mjData* d) {
    mjv_updateScene(m_, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
}

void Viewer::add_ghost(mjData* d, std::array<float, 4> rgba) {
    const int first = scn_.ngeom;
    mjv_addGeoms(m_, d, &opt_, nullptr, mjCAT_DYNAMIC, &scn_);
    for (int i = first; i < scn_.ngeom; ++i) {
        mjvGeom& g = scn_.geoms[i];
        for (int k = 0; k < 4; ++k) g.rgba[k] = rgba[k];
        g.matid = -1;           // plain colour, no texture
        g.transparent = 1;
        g.reflectance = 0;
    }
}

void Viewer::end_frame() {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjr_render(viewport, &scn_, &con_);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
}

// --- input ------------------------------------------------------------------------
Viewer* Viewer::self(GLFWwindow* w) { return static_cast<Viewer*>(glfwGetWindowUserPointer(w)); }

void Viewer::set_camera(int index) {
    camera_index_ = index;
    if (index < 0) {
        cam_.type = mjCAMERA_FREE;
    } else {
        cam_.type = mjCAMERA_FIXED;
        cam_.fixedcamid = index;
    }
}

void Viewer::on_key(GLFWwindow* w, int key, int, int action, int) {
    if (action != GLFW_PRESS || ImGui::GetIO().WantCaptureKeyboard) return;
    Viewer* v = self(w);
    switch (key) {
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(w, 1); break;
        case GLFW_KEY_C: {
            int next = v->camera_index_ + 1;
            v->set_camera(next >= v->m_->ncam ? -1 : next);
            std::printf("camera: %s\n", v->camera_index_ < 0 ? "free"
                                        : mj_id2name(v->m_, mjOBJ_CAMERA, v->camera_index_));
            break;
        }
        default: break;
    }
}

void Viewer::on_button(GLFWwindow* w, int, int, int) {
    Viewer* v = self(w);
    const bool imgui = ImGui::GetIO().WantCaptureMouse;   // a press on a panel doesn't drag the camera
    v->button_left_ = !imgui && glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    v->button_right_ = !imgui && glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(w, &v->last_x_, &v->last_y_);
}

void Viewer::on_move(GLFWwindow* w, double x, double y) {
    Viewer* v = self(w);
    if (!v->button_left_ && !v->button_right_) return;
    const double dx = x - v->last_x_, dy = y - v->last_y_;
    v->last_x_ = x;
    v->last_y_ = y;
    if (v->camera_index_ >= 0) return;   // only the free camera moves
    int width, height;
    glfwGetWindowSize(w, &width, &height);
    const bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    const mjtMouse a = v->button_right_ ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                                        : (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V);
    mjv_moveCamera(v->m_, a, dx / height, dy / height, &v->cam_);   // MuJoCo 4.0 signature
}

void Viewer::on_scroll(GLFWwindow* w, double, double yoff) {
    Viewer* v = self(w);
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (v->camera_index_ < 0) mjv_moveCamera(v->m_, mjMOUSE_ZOOM, 0, -0.05 * yoff, &v->cam_);
}

}  // namespace ec
