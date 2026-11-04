cmake_minimum_required(VERSION 3.16)
project(station_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(elite-cs-series-sdk REQUIRED)        # Elite CS SDK
find_path(ASIO_INCLUDE_DIR asio.hpp REQUIRED)      # standalone asio (libasio-dev)

# ESP32-CAM receiver (standalone asio)
add_library(esp32cam_receiver STATIC src/esp32cam/frame_receiver.cpp)
target_include_directories(esp32cam_receiver PUBLIC ${ASIO_INCLUDE_DIR})
target_compile_definitions(esp32cam_receiver PUBLIC ASIO_STANDALONE)
target_link_libraries(esp32cam_receiver PUBLIC Threads::Threads)
set_target_properties(esp32cam_receiver PROPERTIES POSITION_INDEPENDENT_CODE ON)

# RTSI reader (Elite SDK, Boost.Asio inside; independent from the camera's asio)
add_library(rtsi_reader STATIC src/robot/rtsi_reader.cpp)
target_link_libraries(rtsi_reader PUBLIC elite-cs-series-sdk::shared Threads::Threads)
set_target_properties(rtsi_reader PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(station_bridge_node src/station_bridge_node.cpp)
target_include_directories(station_bridge_node PRIVATE src)
target_link_libraries(station_bridge_node PRIVATE
  esp32cam_receiver rtsi_reader
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

install(TARGETS station_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
