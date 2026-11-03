cmake_minimum_required_version(VERSION 3.22.1)

project(robot_testing)

find_package(elite-cs-series-sdk CONFIG REQUIRED)
find_package(Boost 1.74 REQUIRED)
find_package(Threads REQUIRED)

set(SYSTEM_LIB pthread)

add_subdirectory(connect_robot)
