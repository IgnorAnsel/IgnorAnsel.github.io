cmake_minimum_required(VERSION 3.10)
project(pcl_test)

# 查找 PCL 库
find_package(PCL 1.14 REQUIRED)

include_directories(include)
include_directories(/home/ansel/Orbbec/772534_OpenNI_v2.3.0.85_20220615_1b09bbfd_linux_x64_release/sdk/Include)
link_directories(/home/ansel/Orbbec/772534_OpenNI_v2.3.0.85_20220615_1b09bbfd_linux_x64_release/sdk/libs)
# 添加可执行文件
add_executable(pcl_test src/pcl_test.cpp)
add_executable(PLY2PCD src/PLY2PCD.cpp)
add_executable(input_output_test src/input_output.cpp)
add_executable(point_type src/point_type.cpp)
add_executable(kdsearch src/kdsearch.cpp)
add_executable(kdsearch_flann src/kdsearchflann.cpp)
add_executable(PCLVisualizer src/PCLVisualizer.cpp)
add_executable(filter_US src/filter_uniformsampling.cpp)
add_executable(filter_VG src/filter_voxelgrid.cpp)
add_executable(bilateralFilter src/bilateralFilter.cpp)
add_executable(faceNormal src/faceNormal.cpp)

# 添加关键点检测的可执行文件
add_executable(ISSKeypoint3D src/KeyPoint/ISSKeypoint3D.cpp)
add_executable(HarrisKeypoint3D src/KeyPoint/HarrisKeypoint3D.cpp)
add_executable(SIFTKeypoint src/KeyPoint/SIFTKeypoint.cpp)

#
add_executable(PFH src/Descriptors/PFH.cpp)
add_executable(FPFH src/Descriptors/FPFH.cpp)
add_executable(SHOT src/Descriptors/SHOT.cpp)
add_executable(RoPS src/Descriptors/RoPS.cpp)

#
add_executable(Fullinstance src/Fullinstance.cpp)
#
add_executable(OrbbecTest src/OrbbecTest.cpp)
#
add_executable(ICP_example src/ICP_example.cpp)

#
add_executable(Rotate_pcl src/pclHandler/Rotate_pcl.cpp)
add_executable(Add_pcl src/pclHandler/Add_pcl.cpp)

#
add_executable(GreedyProjection src/SurfaceReconstruction/GreedyProjection.cpp)
add_executable(Poisson src/SurfaceReconstruction/Poisson.cpp)
add_executable(MarchingCubes src/SurfaceReconstruction/MarchingCubes.cpp)
add_executable(MLS src/SurfaceReconstruction/MLS.cpp)
# 将所有可执行文件放入列表中
set(EXECUTABLES pcl_test PLY2PCD input_output_test point_type kdsearch kdsearch_flann 
PCLVisualizer filter_US filter_VG bilateralFilter faceNormal ISSKeypoint3D HarrisKeypoint3D SIFTKeypoint 
PFH FPFH SHOT RoPS
Fullinstance
OrbbecTest
ICP_example
Rotate_pcl Add_pcl
GreedyProjection Poisson MarchingCubes MLS)

# 链接 PCL 库和包含目录到每个可执行文件
foreach(exe ${EXECUTABLES})
    target_link_libraries(${exe} ${PCL_LIBRARIES} OpenNI2)
    target_include_directories(${exe} PRIVATE ${PCL_INCLUDE_DIRS} include)
endforeach()
