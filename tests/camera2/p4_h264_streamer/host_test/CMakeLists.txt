# PC test of the firmware's stream path (no board needed):
#   cmake -S host_test -B build-host && cmake --build build-host
#   ffmpeg -f lavfi -i testsrc2=size=1280x960:rate=30 -frames:v 120 -c:v libx264 -profile:v baseline \
#          -preset ultrafast -tune zerolatency -g 30 -x264-params repeat-headers=1 -f h264 test.h264
#   ./build-host/stream_check test.h264
cmake_minimum_required(VERSION 3.22)
project(p4_stream_check CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIB ${CMAKE_CURRENT_LIST_DIR}/../../../../libraries)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
find_package(Threads REQUIRED)

add_executable(stream_check stream_check.cc ../main/p4_stamp.cc
               ${LIB}/camera/stream_format.cc ${LIB}/camera/server.cc ${LIB}/camera/errors.cc
               ${LIB}/camera/client.cc ${LIB}/camera/decoder.cc)
target_include_directories(stream_check PRIVATE ${LIB} ../main)
target_compile_definitions(stream_check PRIVATE ASIO_NO_DEPRECATED)
target_compile_options(stream_check PRIVATE -Wall -Wextra -Wno-mismatched-new-delete)
target_link_libraries(stream_check PRIVATE PkgConfig::FFMPEG Threads::Threads)
