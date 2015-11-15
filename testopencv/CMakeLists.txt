project(MAIN)  
#message(STATUS "project source: " ${MAIN_SOURCE_DIR}) 
#message(STATUS "project binary: " ${MAIN_BINARY_DIR})
cmake_minimum_required(VERSION 2.8)
find_package(OpenCV REQUIRED)  
add_subdirectory(include)
include_directories(${MAIN_SOURCE_DIR}/include)
link_directories(${INCLUDE_BINARY_DIR}/include)

aux_source_directory(. DIR_SRCS)
add_executable(execute ${DIR_SRCS})

target_link_libraries(execute ${OpenCV_LIBS} include)


