// GLFW window with a MuJoCo scene, a mouse-driven camera and Dear ImGui on top.
// Shared by the simulator and the digital twin.
//
//   ec::Viewer viewer(model, "title");
//   while (!viewer.should_close()) {
//       viewer.begin_frame();          // poll events, start the ImGui frame
//       ImGui::Begin("panel"); ... ImGui::End();
//       viewer.update_scene(data);     // copy the simulation state into the scene
//       viewer.end_frame();            // render scene + ImGui, swap
//   }
//
// Mouse: left drag rotates, right drag moves, shift for the horizontal variant,
// wheel zooms (free camera only). Keys: c cycles cameras, esc quits.
// The mouse and keyboard go to ImGui while it wants them (hovering a panel, typing).
#pragma once

#include <mujoco/mujoco.h>

#include <array>

struct GLFWwindow;

namespace ec {

class Viewer {
public:
    Viewer(const mjModel* m, const char* title, int width = 1400, int height = 900);
    ~Viewer();
    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    bool should_close() const;
    void close();

    void begin_frame();
    void update_scene(mjData* d);
    // Draw the geoms of a second state of the same model, tinted (e.g. a command target)
    void add_ghost(mjData* d, std::array<float, 4> rgba = {0.2f, 0.9f, 0.3f, 0.35f});
    void end_frame();

    mjvOption& option() { return opt_; }
    mjvCamera& camera() { return cam_; }
    GLFWwindow* window() { return window_; }

private:
    static Viewer* self(GLFWwindow* w);
    static void on_key(GLFWwindow* w, int key, int scancode, int action, int mods);
    static void on_button(GLFWwindow* w, int button, int action, int mods);
    static void on_move(GLFWwindow* w, double x, double y);
    static void on_scroll(GLFWwindow* w, double xoff, double yoff);
    void set_camera(int index);

    const mjModel* m_;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    int camera_index_ = -1;   // -1 = free camera, else fixed camera id
    bool button_left_ = false, button_right_ = false;
    double last_x_ = 0, last_y_ = 0;
};

}  // namespace ec
