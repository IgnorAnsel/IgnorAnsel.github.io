cmake_minimum_required(VERSION 3.10)
project(pcl_test)

find_package(PCL 1.14 REQUIRED)

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${PCL_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS})
