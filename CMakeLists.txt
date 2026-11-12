cmake_minimum_required(VERSION 3.22)

project(robotsimulator VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)   # the robot client uses coroutines (asio::awaitable)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# System packages:
#   sudo apt install build-essential cmake git libglfw3-dev libasio-dev nlohmann-json3-dev libegl-dev libgl-dev \
#                    libavcodec-dev libswscale-dev libavutil-dev pkg-config
#   sudo apt install libompl-dev
#   ROS 2 (optional): see the ROS section below
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

add_subdirectory(libraries)

# ---------------------------------------------------------------------------
# ROS 2 (optional): the twin's ROS interface and the planner's action client.
#   No `source setup.bash` needed: CMake only needs the install's prefix, and the
#   programs set up their own ROS environment at start (library/ros_env.h).
#   EC_ROS_DIR defaults to /opt/ros/$ROS_DISTRO, else /opt/ros/humble, else jazzy.
#   -DEC_WITH_ROS=OFF builds everything without ROS.
#   Needs: rclcpp rclcpp_action sensor_msgs std_srvs diagnostic_msgs control_msgs
#     sudo apt install ros-humble-control-msgs     (the only one not in ros-humble-ros-base)
# ---------------------------------------------------------------------------
option(EC_WITH_ROS "Build the ROS 2 interfaces when ROS 2 is installed" ON)
if(EC_WITH_ROS AND NOT EC_ROS_DIR)
    foreach(_candidate "/opt/ros/$ENV{ROS_DISTRO}" /opt/ros/humble /opt/ros/jazzy)
        if(EXISTS "${_candidate}/share/rclcpp")
            set(EC_ROS_DIR "${_candidate}" CACHE PATH "ROS 2 install prefix")
            break()
        endif()
    endforeach()
endif()
if(EC_WITH_ROS AND EC_ROS_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${EC_ROS_DIR}")   # after the other packages were found
    # ROS's CMake scripts look up type supports through AMENT_PREFIX_PATH: set it for this
    # CMake run only (it doesn't leak into your shell)
    if(NOT DEFINED ENV{AMENT_PREFIX_PATH})
        set(ENV{AMENT_PREFIX_PATH} "${EC_ROS_DIR}")
    endif()
    # ...and they run Python helpers that import ROS's own modules (ament_package), which
    # setup.bash would put on PYTHONPATH: /opt/ros/<distro>/lib/python3.X/site-packages
    file(GLOB _ec_ros_python LIST_DIRECTORIES true
         "${EC_ROS_DIR}/lib/python3*/site-packages" "${EC_ROS_DIR}/local/lib/python3*/dist-packages")
    if(_ec_ros_python)
        string(REPLACE ";" ":" _ec_ros_python "${_ec_ros_python}")
        if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
            set(ENV{PYTHONPATH} "${_ec_ros_python}:$ENV{PYTHONPATH}")
        else()
            set(ENV{PYTHONPATH} "${_ec_ros_python}")
        endif()
    endif()
    set(_ec_ros_packages rclcpp rclcpp_action sensor_msgs std_srvs diagnostic_msgs control_msgs)
    set(_ec_ros_missing "")
    foreach(_pkg IN LISTS _ec_ros_packages)
        find_package(${_pkg} QUIET)
        if(NOT ${_pkg}_FOUND)
            list(APPEND _ec_ros_missing ${_pkg})
        endif()
    endforeach()
    if(_ec_ros_missing)
        message(WARNING "ROS 2 at ${EC_ROS_DIR} lacks: ${_ec_ros_missing}. Building without ROS. "
                        "(control_msgs: sudo apt install ros-<distro>-control-msgs)")
    else()
        add_library(ec_ros INTERFACE)
        target_link_libraries(ec_ros INTERFACE rclcpp::rclcpp rclcpp_action::rclcpp_action
            ${sensor_msgs_TARGETS} ${std_srvs_TARGETS} ${diagnostic_msgs_TARGETS} ${control_msgs_TARGETS})
        target_compile_definitions(ec_ros INTERFACE EC_HAVE_ROS EC_ROS_DIR="${EC_ROS_DIR}")
        target_include_directories(ec_ros INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/library)
        # Find ROS's libraries at startup without LD_LIBRARY_PATH. The apt-built ROS libraries
        # carry no search path of their own, and a RUNPATH (CMake's default) only covers the
        # program's DIRECT dependencies, so librclcpp's own dependencies would not be found.
        # The older DT_RPATH (--disable-new-dtags) applies to the whole dependency tree.
        set(_ec_ros_libdirs "")
        foreach(_dir "${EC_ROS_DIR}/lib" "${EC_ROS_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
            if(EXISTS "${_dir}")
                list(APPEND _ec_ros_libdirs "LINKER:-rpath,${_dir}")
            endif()
        endforeach()
        target_link_options(ec_ros INTERFACE "LINKER:--disable-new-dtags" ${_ec_ros_libdirs})
        message(STATUS "ROS 2 from ${EC_ROS_DIR}: building the ROS interfaces")
    endif()
elseif(EC_WITH_ROS)
    message(STATUS "ROS 2 not found: building without it (set -DEC_ROS_DIR=/opt/ros/<distro>)")
endif()

add_subdirectory(applications)
