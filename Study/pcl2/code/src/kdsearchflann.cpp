#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/flann_search.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile<PointT>(argv[1], *cloud);

    pcl::search::FlannSearch<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree.nearestKSearch(cloud->points[0], 10, indices, distances);
    std::cout << "Nearest 2 points to the first point: " << std::endl;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        std::cout << "Index: " << indices[i] << ", Distance: " << distances[i] << std::endl;
    }
    return 0;
}