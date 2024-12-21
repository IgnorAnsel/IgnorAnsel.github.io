cmake_minimum_required( VERSION 3.10)
project(OpenCV)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(opencv_test src/opencv_test.cpp)
target_link_libraries(opencv_test ${OpenCV_LIBS})