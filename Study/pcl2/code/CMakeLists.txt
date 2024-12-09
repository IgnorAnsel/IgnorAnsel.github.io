cmake_minimum_required(VERSION 3.10)
project(pcl_test)

# 查找 PCL 库
find_package(PCL 1.14 REQUIRED)

include_directories(include)

# 添加可执行文件
add_executable(pcl_test src/pcl_test.cpp)
add_executable(input_output_test src/input_output.cpp)
add_executable(point_type src/point_type.cpp)
add_executable(kdsearch src/kdsearch.cpp)
add_executable(kdsearch_flann src/kdsearchflann.cpp)
add_executable(PCLVisualizer src/PCLVisualizer.cpp)
add_executable(filter_US src/filter_uniformsampling.cpp)
add_executable(filter_VG src/filter_voxelgrid.cpp)
add_executable(bilateralFilter src/bilateralFilter.cpp)
add_executable(faceNormal src/faceNormal.cpp)
add_executable(ISSKeypoint3D src/ISSKeypoint3D.cpp)
add_executable(HarrisKeypoint3D src/HarrisKeypoint3D.cpp)
# 将所有可执行文件放入列表中
set(EXECUTABLES pcl_test input_output_test point_type kdsearch kdsearch_flann 
PCLVisualizer filter_US filter_VG bilateralFilter faceNormal ISSKeypoint3D HarrisKeypoint3D)

# 链接 PCL 库和包含目录到每个可执行文件
foreach(exe ${EXECUTABLES})
    target_link_libraries(${exe} ${PCL_LIBRARIES})
    target_include_directories(${exe} PRIVATE ${PCL_INCLUDE_DIRS} include)
endforeach()
