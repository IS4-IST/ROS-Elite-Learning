add_executable(simulator simulator.cc)
target_link_libraries(simulator PRIVATE ec_common ec_emulator asio::asio ec_camera)
target_compile_options(simulator PRIVATE -Wall -Wextra)
