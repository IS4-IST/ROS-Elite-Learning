configure_file(${PROJECT_SOURCE_DIR}/resources/external_control.script ${CMAKE_CURRENT_BINARY_DIR}/ COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/resources/input_recipe.txt ${CMAKE_CURRENT_BINARY_DIR}/ COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}/resources/output_recipe.txt ${CMAKE_CURRENT_BINARY_DIR}/ COPYONLY)

add_executable(connect_example main.cpp)

target_link_libraries(
    connect_example
    elite-cs-series-sdk::shared
    Boost::headers
    Boost::program_options
    Threads::Threads
)
