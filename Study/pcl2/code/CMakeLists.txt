cmake_minimum_required(VERSION 3.10)
project(pcl_test)

# 查找 PCL 库
find_package(PCL 1.14 REQUIRED)




# 添加可执行文件
add_executable(pcl_test src/pcl_test.cpp)
add_executable(input_output_test src/input_output.cpp)
add_executable(point_type src/point_type.cpp)
add_executable(kdsearch src/kdsearch.cpp)
add_executable(kdsearch_flann src/kdsearchflann.cpp)
add_executable(PCLVisualizer src/PCLVisualizer.cpp)
# 将所有可执行文件放入列表中
set(EXECUTABLES pcl_test input_output_test point_type kdsearch kdsearch_flann PCLVisualizer)

# 链接 PCL 库和包含目录到每个可执行文件
foreach(exe ${EXECUTABLES})
    target_link_libraries(${exe} ${PCL_LIBRARIES})
    target_include_directories(${exe} PRIVATE ${PCL_INCLUDE_DIRS})
endforeach()
