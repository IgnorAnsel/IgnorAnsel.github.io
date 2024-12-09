#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/keypoints/iss_3d.h>

#include "resolution.h" // 用于计算点云分辨率

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    // 加载点云数据
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPLYFile<PointT>(argv[1], *cloud);
    std::cout << "origin cloud size: " << cloud->size() << std::endl;
    // 计算点云分辨率
    float resolution = computeCloudResolution(cloud);
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

    pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>);
    iss.compute(*keypoints);
    std::cout << "keypoints size: " << keypoints->size() << std::endl;

    return 0;
}