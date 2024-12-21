#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud3(new pcl::PointCloud<PointT>);

    // 读取点云数据
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud1) == -1)
    {
        PCL_ERROR("Couldn't read file1 \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<PointT>(argv[2], *cloud2) == -1)
    {
        PCL_ERROR("Couldn't read file2 \n");
        return (-1);
    }

    // 将两个点云数据相加
    *cloud3 = *cloud1 + *cloud2;

    // 保存结果
    pcl::io::savePCDFileASCII("result.pcd", *cloud3);
}