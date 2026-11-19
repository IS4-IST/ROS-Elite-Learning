# The stream format and the TCP server are the repo's own (libraries/camera), so the P4
# serves byte-for-byte what the simulator serves. encoder.h is used for its EncodedFrame
# struct only; nothing here needs FFmpeg.
set(LIB ${CMAKE_CURRENT_LIST_DIR}/../../../../libraries)

set(reqs esp_video esp_h264 asio esp_eth esp_netif esp_event nvs_flash esp_timer lwip)
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    list(APPEND reqs ip101)            # the PHY drivers moved out of esp_eth in IDF 6
endif()

# WHOLE_ARCHIVE: asio_noexcept.cc is only referenced by the asio library (linked after main)
idf_component_register(SRCS "main.cc" "p4_camera.cc" "p4_stamp.cc" "asio_noexcept.cc"
                            "${LIB}/camera/stream_format.cc" "${LIB}/camera/server.cc" "${LIB}/camera/errors.cc"
                       INCLUDE_DIRS "${LIB}"
                       PRIV_REQUIRES ${reqs}
                       WHOLE_ARCHIVE)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-mismatched-new-delete)
