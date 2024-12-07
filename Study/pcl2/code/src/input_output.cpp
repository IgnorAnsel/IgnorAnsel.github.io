#include <pcl/io/pcd_io.h> // PCL 库中用于读写 PCD 文件的头文件
#include <pcl/point_types.h> // PCL 库中定义点类型（如 PointXYZ）的头文件
#include <iostream>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT> *cloud2 = new pcl::PointCloud<PointT>;
    if (pcl::io::loadPCDFile<PointT>("./random_filtered_cloud.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file bunny.pcd \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<PointT>("./random_filtered_cloud.pcd", *cloud2) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file bunny.pcd \n");
        return (-1);
    }
    std::cout << "点云大小" << cloud->points.size() << " data points." << std::endl;
    std::cout << "点云大小" << cloud2->points.size() << " data points." << std::endl;
    pcl::io::savePCDFileASCII("output.pcd", *cloud);
    pcl::io::savePCDFileASCII("output2.pcd", *cloud2);
    return 0;
}