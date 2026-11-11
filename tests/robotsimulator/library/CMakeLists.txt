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
# ec_robot: async C++ client for the controller, on your io_context (C++20 coroutines)
#   rpc_client (8055 JSON-RPC), monitor (8056 status), robot (operations), streamer
#   (transparent transmission). Expected failures are std::error_code, not exceptions.
#   Headers are included as "ec_robot/robot.h".
# ---------------------------------------------------------------------------
add_library(ec_robot STATIC
    ec_robot/types.cc
    ec_robot/rpc_client.cc
    ec_robot/monitor.cc
    ec_robot/robot.cc
    ec_robot/streamer.cc
)
target_include_directories(ec_robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_robot PUBLIC asio::asio nlohmann_json::nlohmann_json Threads::Threads)
target_compile_options(ec_robot PRIVATE -Wall -Wextra)
# GCC 12/13 warn about asio's coroutine frame allocation (a false positive); everyone who
# instantiates the client's coroutines would see it, so it's PUBLIC
target_compile_options(ec_robot PUBLIC $<$<CXX_COMPILER_ID:GNU>:-Wno-mismatched-new-delete>)

# Tests against the simulator: start `simulator --headless` first, then run them
option(EC_BUILD_TESTS "Build the ec_robot tests (they need a running simulator)" ON)
if(EC_BUILD_TESTS)
    foreach(_test rpc monitor robot stream)
        add_executable(test_${_test} ec_robot/tests/test_${_test}.cc)
        target_link_libraries(test_${_test} PRIVATE ec_robot)
    endforeach()
endif()

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

# ---------------------------------------------------------------------------
# ec_planning: OMPL joint-space planner with MuJoCo as the collision checker.
#   The twin uses it to check every motion before it is sent, so it is required.
#   OMPL 1.5 (Ubuntu 22.04/24.04 apt) only sets OMPL_* variables; OMPL >= 1.7
#   exports ompl::ompl. Both are handled.
# ---------------------------------------------------------------------------
find_package(ompl REQUIRED)   # sudo apt install libompl-dev
if(NOT TARGET ompl::ompl)
    # OMPL 1.5's include list names folders that may not exist (OpenMPI, HDF5): keep the real ones
    set(_ompl_includes "")
    foreach(_dir IN LISTS OMPL_INCLUDE_DIRS)
        if(EXISTS "${_dir}")
            list(APPEND _ompl_includes "${_dir}")
        endif()
    endforeach()
    # ...and its library list adds Boost dev symlinks Ubuntu's libompl-dev doesn't install;
    # libompl.so already links Boost itself, so only existing files are kept
    set(_ompl_libs "")
    foreach(_lib IN LISTS OMPL_LIBRARIES)
        if(EXISTS "${_lib}")
            list(APPEND _ompl_libs "${_lib}")
        endif()
    endforeach()
    add_library(ompl::ompl INTERFACE IMPORTED)
    target_include_directories(ompl::ompl INTERFACE ${_ompl_includes})
    target_link_libraries(ompl::ompl INTERFACE ${_ompl_libs})
endif()
add_library(ec_planning STATIC planning/planner.cc)
target_include_directories(ec_planning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ec_planning PUBLIC mujoco::mujoco ompl::ompl)
target_compile_options(ec_planning PRIVATE -Wall -Wextra)
