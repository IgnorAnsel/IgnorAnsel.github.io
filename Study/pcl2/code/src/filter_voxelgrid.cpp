#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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

    // 使用体素栅格滤波器进行降采样
    pcl::VoxelGrid<PointT> vg; // 创建滤波器对象
    vg.setInputCloud(cloud);  // 设置输入点云
    const float leaf = 0.05f; // 设置体素栅格的尺寸
    vg.setLeafSize(leaf, leaf, leaf); // 设置体素栅格的体积
    vg.filter(*cloud_filtered); // 执行滤波操作
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;
    return 0;
}