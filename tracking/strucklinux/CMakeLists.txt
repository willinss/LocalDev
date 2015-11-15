cmake_minimum_required(VERSION 2.8) 
project(MAIN) 
find_package(OpenCV REQUIRED) 
add_subdirectory(GraphUtils)
include_directories(${MAIN_SOURCE_DIR}/GraphUtils)
link_directories(${GRAPHUTILS_BINARY_DIR}/GraphUtils)

aux_source_directory(. DIR_SRCS)
add_executable(test ${DIR_SRCS})

target_link_libraries(test ${OpenCV_LIBS} GraphUtils)
