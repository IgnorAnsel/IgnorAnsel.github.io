#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/keypoints/harris_3d.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPLYFile<PointT>(argv[1], *cloud);

    std::cout << "original cloud size: " << cloud->size() << std::endl;

    double resolution = computeCloudResolution(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> harris;
    harris.setNonMaxSupression(true);
    harris.setRadiusSearch(10 * resolution);
    harris.setThreshold(1E-6);
    harris.setSearchMethod(tree);
    harris.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    harris.compute(*keypoints);

    pcl::console::print_highlight("Detected %d points !\n", keypoints->size());
    return 0;
}