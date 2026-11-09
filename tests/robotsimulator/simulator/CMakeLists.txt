add_executable(simulator simulator.cc)
target_link_libraries(simulator PRIVATE ec_common ec_emulator asio::asio)
target_compile_options(simulator PRIVATE -Wall -Wextra)
