# ---------------------------------------------------------------------------
# Dear ImGui with the GLFW + OpenGL3 backends
# ---------------------------------------------------------------------------
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw ${CMAKE_DL_LIBS})

# ---------------------------------------------------------------------------
# ec_common: model loading, helpers, and the GLFW + MuJoCo + ImGui window
# ---------------------------------------------------------------------------
add_library(ec_common STATIC
    common.cc
    viewer.cc
)
target_include_directories(ec_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_common PUBLIC mujoco::mujoco imgui glfw)
target_compile_definitions(ec_common PUBLIC EC_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/models")
target_compile_options(ec_common PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# ec_egl: headless OpenGL context for offscreen rendering (no window, no X server).
#   Kept separate on purpose: MuJoCo picks its OpenGL loader from the libraries
#   loaded in the process, so a program must link EITHER this (EGL + libOpenGL)
#   OR a GLFW window, never both. Nothing links it yet.
# ---------------------------------------------------------------------------
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL COMPONENTS OpenGL EGL)
if(OpenGL_EGL_FOUND)
    add_library(ec_egl STATIC egl_context.cc)
    target_include_directories(ec_egl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(ec_egl PUBLIC mujoco::mujoco OpenGL::EGL OpenGL::OpenGL)
endif()

# ---------------------------------------------------------------------------
# ec_robot: C++ client for the controller (8055 JSON-RPC, 8056 monitor, streaming)
#   Same code as ec_robot_cpp; headers are included as "ec_robot/robot.hpp".
# ---------------------------------------------------------------------------
add_library(ec_robot STATIC
    ec_robot/rpc_client.cc
    ec_robot/monitor.cc
    ec_robot/streamer.cc
    ec_robot/robot.cc
)
target_include_directories(ec_robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_robot PUBLIC asio::asio nlohmann_json::nlohmann_json Threads::Threads)
target_compile_options(ec_robot PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# ec_emulator: the controller's state machine + servers on top of MuJoCo
# ---------------------------------------------------------------------------
add_library(ec_emulator STATIC
    emulator/packet.cc
    emulator/kinematics.cc
    emulator/motion.cc
    emulator/controller.cc
    emulator/servers.cc
)
target_include_directories(ec_emulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_emulator PUBLIC mujoco::mujoco asio::asio nlohmann_json::nlohmann_json Threads::Threads)
target_compile_options(ec_emulator PRIVATE -Wall -Wextra)
