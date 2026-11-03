configure_file(resource/external_control.script ${PROJECT_BINARY_DIR}/connect_robot/ COPYONLY)
configure_file(resource/input_recipe.txt ${PROJECT_BINARY_DIR}/connect_robot/ COPYONLY)
configure_file(resource/output_recipe.txt ${PROJECT_BINARY_DIR}/connect_robot/ COPYONLY)

if(ELITE_COMPILE_KIN_PLUGIN)
    add_definitions(-DELITE_COMPILE_KIN_PLUGIN)
endif()

if(ELITE_COMPILE_POSE_ALG_PLUGIN)
    add_definitions(-DELITE_COMPILE_POSE_ALG_PLUGIN)
endif()

add_executable(connect_example main.cpp)

target_link_libraries(
    connect_example
    elite_cs_series_sdk::shared
    ${SYSTEM_LIB}
    Boost::program_options
)
target_link_directories(
    connect_example
    PRIVATE 
    ${CMAKE_BINARY_DIR}
    ${Boost_LIBRARY_DIRS}
)
target_include_directories(
    connect_example
    PRIVATE 
    ${Boost_INCLUDE_DIRS}
)