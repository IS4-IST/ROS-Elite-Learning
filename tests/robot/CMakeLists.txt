cmake_minimum_required(VERSION 3.22.1)

project(robot_testing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(elite-cs-series-sdk CONFIG REQUIRED)
find_package(Boost 1.74 REQUIRED COMPONENTS program_options)
find_package(Threads REQUIRED)

add_subdirectory(connect_robot)
