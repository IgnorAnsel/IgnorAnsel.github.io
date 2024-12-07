#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file bun0.pcd \n");
        return (-1);
    }

    // 使用UniformSampling滤波器
    pcl::UniformSampling<PointT> us ; // 创建滤波器对象
    us.setInputCloud(cloud);  // 设置输入点云
    double radius = 0.01; // 设置体素栅格的半径
    us.setRadiusSearch(radius); // 设置搜索半径
    us.filter(*cloud_filtered); // 执行滤波操作
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;
    return 0;
}