cmake_minimum_required(VERSION 3.22)

project(robotsimulator VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# System packages:
#   sudo apt install build-essential cmake git libglfw3-dev libasio-dev nlohmann-json3-dev libegl-dev libgl-dev
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(nlohmann_json 3.10 REQUIRED)

# Standalone Asio is header-only and ships no CMake package
find_path(ASIO_INCLUDE_DIR asio.hpp REQUIRED)
add_library(asio::asio INTERFACE IMPORTED)
target_include_directories(asio::asio INTERFACE ${ASIO_INCLUDE_DIR})
target_compile_definitions(asio::asio INTERFACE ASIO_STANDALONE ASIO_NO_DEPRECATED)
target_link_libraries(asio::asio INTERFACE Threads::Threads)

# ---------------------------------------------------------------------------
# MuJoCo, built from source at a pinned release
# ---------------------------------------------------------------------------
include(FetchContent)
set(MUJOCO_BUILD_EXAMPLES     OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE     OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_STUDIO       OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS        OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL   OFF CACHE BOOL "" FORCE)
set(MUJOCO_USE_FILAMENT       OFF CACHE BOOL "" FORCE)   # classic OpenGL renderer
FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG        4.0.0
    GIT_SHALLOW    TRUE
)

# Dear ImGui (no CMake build of its own: the target is defined in library/)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.9
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(mujoco imgui)

add_subdirectory(library)
add_subdirectory(simulator)
add_subdirectory(digitaltwin)
