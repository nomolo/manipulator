cmake_minimum_required(VERSION 3.15)
project(manipulator_simulate)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

find_package(glfw3 REQUIRED)

include_directories(include)
link_directories(lib)

aux_source_directory(src SRC_FILE)
aux_source_directory(src/controller CTRL_FILE)

add_executable(manipulator ${SRC_FILE} ${CTRL_FILE})
target_link_libraries(manipulator mujoco glfw jsoncpp)