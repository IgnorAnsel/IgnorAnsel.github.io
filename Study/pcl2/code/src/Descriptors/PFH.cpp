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