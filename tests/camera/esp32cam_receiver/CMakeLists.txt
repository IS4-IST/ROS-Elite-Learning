cmake_minimum_required(VERSION 3.16)
project(esp32cam_receiver CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Standalone Asio (Ubuntu/Debian: sudo apt install libasio-dev)
find_path(ASIO_INCLUDE_DIR asio.hpp REQUIRED)
find_package(Threads REQUIRED)

# Receiver library: no display, no OpenCV. Reuse it later in a ROS node or elsewhere.
add_library(esp32cam_receiver src/frame_receiver.cpp)
target_include_directories(esp32cam_receiver PUBLIC src ${ASIO_INCLUDE_DIR})
target_compile_definitions(esp32cam_receiver PUBLIC ASIO_STANDALONE)
target_link_libraries(esp32cam_receiver PUBLIC Threads::Threads)

# Viewer: shows the stream with OpenCV (Ubuntu/Debian: sudo apt install libopencv-dev)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc highgui)
add_executable(esp32cam_viewer src/main.cpp)
target_link_libraries(esp32cam_viewer PRIVATE esp32cam_receiver ${OpenCV_LIBS})
target_include_directories(esp32cam_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})
