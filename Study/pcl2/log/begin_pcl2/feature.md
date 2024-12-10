@[9.点云特征描述]()

# 点云特镇征描述

## 1.常用点云特征描述方法

### 1.1 PFH

#### 1.1.1 简介

PFH描述子用于描述点云中局部几何形状的特征，通过点云的法线信息以及点间的空间关系来捕捉每个点的几何信息。
#### 1.1.2 代码
```cpp
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::PFHSignature125 FeatureT;
pcl::PointCloud<PointT>::Ptr GetKeyPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    double resolution = computeCloudResolution(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::ISSKeypoint3D<PointT, PointT> iss;
    iss.setSearchMethod(tree);
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.setNumberOfThreads(4);
    iss.setInputCloud(cloud);

    iss.compute(*key_cloud);
    return key_cloud;
}
int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointNT>::Ptr cloud_normals(new pcl::PointCloud<PointNT>);

    if (pcl::io::loadPLYFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return (-1);
    }
    double resolution = computeCloudResolution(cloud);
    key_cloud = GetKeyPointCloud(cloud);
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    ne.setSearchSurface(cloud);
    ne.compute(*cloud_normals);
    std::cout << "compute normal\n";

    // 关键点计算PFH描述子
    pcl::PointCloud<FeatureT>::Ptr cloud_features(new pcl::PointCloud<FeatureT>);
    pcl::PFHEstimation<PointT, PointNT, FeatureT> pfh_est;
    pfh_est.setInputCloud(key_cloud);
    pfh_est.setInputNormals(cloud_normals);
    pfh_est.setSearchSurface(cloud);
    pfh_est.setRadiusSearch(18 * resolution);
    pfh_est.compute(*cloud_features);
    std::cout << "compute pfh\n";
    return 0;

}
```

### 1.2 FPFH


#### 1.2.1 简介
FPFH描述子是PFH的改进版本，减少了计算的复杂度，使得FPFH可以在大规模点云数据集上高效使用。


#### 1.2.2 代码
```cpp
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::FPFHSignature33 FeatureT;
pcl::PointCloud<PointT>::Ptr GetKeyPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    double resolution = computeCloudResolution(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::ISSKeypoint3D<PointT, PointT> iss;
    iss.setSearchMethod(tree);
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.setNumberOfThreads(4);
    iss.setInputCloud(cloud);

    iss.compute(*key_cloud);
    return key_cloud;
}
int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointNT>::Ptr cloud_normals(new pcl::PointCloud<PointNT>);

    if (pcl::io::loadPLYFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return (-1);
    }
    double resolution = computeCloudResolution(cloud);
    key_cloud = GetKeyPointCloud(cloud);
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    ne.setSearchSurface(cloud);
    ne.compute(*cloud_normals);
    std::cout << "compute normal\n";

    // 关键点计算PFH描述子
    pcl::PointCloud<FeatureT>::Ptr cloud_features(new pcl::PointCloud<FeatureT>);
    pcl::FPFHEstimation<PointT, PointNT, FeatureT> fpfh_est;
    fpfh_est.setInputCloud(key_cloud);
    fpfh_est.setInputNormals(cloud_normals);
    fpfh_est.setSearchSurface(cloud);
    fpfh_est.setRadiusSearch(18 * resolution);
    fpfh_est.compute(*cloud_features);
    std::cout << "compute pfh\n";
    return 0;

}
```

### 1.3 SHOT

#### 1.3.1 简介
SHOT是一种基于点云法线和局部几何形状的描述子，能够捕捉点云的旋转不变性。


#### 1.3.2 代码
```c++
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;
typedef pcl::SHOT352 FeatureT;
pcl::PointCloud<PointT>::Ptr GetKeyPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    double resolution = computeCloudResolution(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::ISSKeypoint3D<PointT, PointT> iss;
    iss.setSearchMethod(tree);
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.setNumberOfThreads(4);
    iss.setInputCloud(cloud);

    iss.compute(*key_cloud);
    return key_cloud;
}
int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointNT>::Ptr cloud_normals(new pcl::PointCloud<PointNT>);

    if (pcl::io::loadPLYFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return (-1);
    }
    double resolution = computeCloudResolution(cloud);
    key_cloud = GetKeyPointCloud(cloud);
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    ne.setSearchSurface(cloud);
    ne.compute(*cloud_normals);
    std::cout << "compute normal\n";

    // 关键点计算shot描述子
    pcl::PointCloud<FeatureT>::Ptr cloud_features(new pcl::PointCloud<FeatureT>);
    pcl::SHOTEstimation<PointT, PointNT, FeatureT> shot;
    shot.setInputCloud(key_cloud);
    shot.setInputNormals(cloud_normals);
    shot.setSearchSurface(cloud);
    shot.setRadiusSearch(18 * resolution);
    //shot.setInputReferenceFrames(lrf); //也可以自己传入局部坐标系
    shot.compute(*cloud_features);
    std::cout << "compute shot\n";
    return 0;

}
```

### 1.4 RoPS

#### 1.4.1 简介
RoPS特征通过对点云的旋转和投影进行统计，生成一个描述点云局部几何结构的特征，常用于物体识别任务。
#### 1.4.2 代码

```c++
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/features/rops_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/keypoints/iss_3d.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NT;
typedef pcl::PointNormal PointNT;
typedef pcl::Histogram<135> FeatureT;
pcl::PointCloud<PointT>::Ptr GetKeyPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    double resolution = computeCloudResolution(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::ISSKeypoint3D<PointT, PointT> iss;
    iss.setSearchMethod(tree);
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.setNumberOfThreads(4);
    iss.setInputCloud(cloud);

    iss.compute(*key_cloud);
    return key_cloud;
}
int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointNT>::Ptr cloud_normals(new pcl::PointCloud<PointNT>);

    if (pcl::io::loadPLYFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return (-1);
    }
    double resolution = computeCloudResolution(cloud);
    key_cloud = GetKeyPointCloud(cloud);
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    ne.setSearchSurface(cloud);
    ne.compute(*cloud_normals);
    std::cout << "compute normal\n";

    // 连接数据
    pcl::PointCloud<PointNT>::Ptr key_cloud_normals(new pcl::PointCloud<PointNT>);
    pcl::concatenateFields(*cloud, *cloud_normals, *key_cloud_normals);

    // ---- rops基于网格，所以要先将pcd点云数据重建网格 ---
    pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT>);
    tree->setInputCloud(key_cloud_normals);
    pcl::GreedyProjectionTriangulation<PointNT> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(key_cloud_normals);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(triangles);

    // rops
    pcl::ROPSEstimation<PointT,FeatureT> rops;
    rops.setInputCloud(key_cloud);
    rops.setSearchSurface(cloud);
    rops.setNumberOfPartitionBins(5);
    rops.setNumberOfRotations(3);
    rops.setRadiusSearch(0.01);
    rops.setSupportRadius(0.01);
    rops.setTriangles(triangles.polygons);
    rops.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree < pcl::PointXYZ>));
    //feature size = number_of_rotations * number_of_axis_to_rotate_around * number_of_projections * number_of_central_moments
    //unsigned int feature_size = number_of_rotations_ * 3 * 3 * 5; //计算出135
    pcl::PointCloud<FeatureT> description;
    rops.compute(description);
    std::cout << "size is " << description.points.size()<<std::endl;
    std::cout << "compute rops\n";
    return 0;

}
```