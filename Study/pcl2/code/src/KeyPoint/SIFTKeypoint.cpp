#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_cloud>" << std::endl;
        return -1;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPLYFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return -1;
    }

    std::cout << "original cloud size: " << cloud->size() << std::endl;

    double resolution = computeCloudResolution(cloud);

    // 法向量
    pcl::NormalEstimation<PointT, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree_n);
    ne.setInputCloud(cloud);
    // ne.setRadiusSearch(resolution * 10);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // 拷贝数据
    for (size_t i = 0; i < cloud_normals->size(); ++i)
    {
        cloud_normals->points[i].x = cloud->points[i].x;
        cloud_normals->points[i].y = cloud->points[i].y;
        cloud_normals->points[i].z = cloud->points[i].z;
    }

    // SIFT参数
    const float min_scale = 0.001f;
    const int n_octaves = 3;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.001f;

    // SIFT关键点
    // 使用法向量作为强度计算关键点，还可以是颜色、强度等
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    
    pcl::PointCloud<pcl::PointWithScale>::Ptr cloud_sift_keypoints(new pcl::PointCloud<pcl::PointWithScale>);
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_normals);
    sift.compute(*cloud_sift_keypoints);
    std::cout << "No of SIFT points in the result are " << cloud_sift_keypoints->points.size() << std::endl;

    return 0;
}