cmake_minimum_required(VERSION 3.22)

set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p4_h264_streamer)
