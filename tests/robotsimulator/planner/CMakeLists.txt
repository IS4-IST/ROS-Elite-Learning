add_executable(planner planner.cc)
target_link_libraries(planner PRIVATE ec_common ec_planning)
target_compile_options(planner PRIVATE -Wall -Wextra)

# Without ROS 2 the planner only plans offline (--start); with it, it reads /joint_states
# and sends motions to the digital twin
if(TARGET ec_ros)
    target_link_libraries(planner PRIVATE ec_ros)
endif()
