add_executable(digitaltwin digitaltwin.cc)
target_link_libraries(digitaltwin PRIVATE ec_common ec_robot asio::asio)
target_compile_options(digitaltwin PRIVATE -Wall -Wextra)
