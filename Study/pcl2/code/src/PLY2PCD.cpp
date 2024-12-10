#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    // Load the PLY file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return (-1);
    }

    // Save the PCD file
    pcl::io::savePCDFileASCII(argv[2], *cloud);
    std::cout << "Saved " << cloud->points.size() << " data points to " << argv[2] << std::endl;
    return (0);
}