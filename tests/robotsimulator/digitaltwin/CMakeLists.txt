add_executable(digitaltwin digitaltwin.cc twin_core.cc)
target_link_libraries(digitaltwin PRIVATE ec_common ec_robot ec_planning asio::asio)
target_compile_options(digitaltwin PRIVATE -Wall -Wextra)

# ROS front end, when ROS 2 was found (top-level CMakeLists.txt)
if(TARGET ec_ros)
    target_sources(digitaltwin PRIVATE ros_interface.cc)
    target_link_libraries(digitaltwin PRIVATE ec_ros)
endif()
