#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
typedef pcl::PointXYZ PointT;
int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("random_filtered_cloud.pcd", *cloud);

    // 创建一个KdTree对象
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud); // 设置要搜索的点云，建立KDTree

    std::vector<int> indices; // 存储查询近邻点索引
    std::vector<float> distances; // 存储近邻点对应距离的平方
    PointT point = cloud->points[0]; // 初始化一个查询点

    kdtree->nearestKSearch(point, 10, indices, distances); // 查询最近邻点
    std::cout << "Nearest 10 points:" << std::endl;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        std::cout << "Point: " << cloud->points[indices[i]].x << " "
                  << cloud->points[indices[i]].y << " "
                  << cloud->points[indices[i]].z << std::endl;
        std::cout << "Distance: " << distances[i] << std::endl;
    }

    double radius = 0.5;

    kdtree->radiusSearch(point, radius, indices, distances); // 查询半径为0.5的近邻点
    std::cout << "Points within radius " << radius << ":" << std::endl;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        std::cout << "Point: " << cloud->points[indices[i]].x << " "
                  << cloud->points[indices[i]].y << " "
                  << cloud->points[indices[i]].z << std::endl;
        std::cout << "Distance: " << distances[i] << std::endl;
    }
    return 0;
}