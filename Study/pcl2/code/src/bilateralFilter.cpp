#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/bilateral.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZI PointT;

void bilateralFilter(pcl::PointCloud<PointT>::Ptr &input, pcl::PointCloud<PointT>::Ptr &output)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // 应用双边滤波器
    pcl::BilateralFilter<PointT> fbf;
    fbf.setInputCloud(input);
    fbf.setSearchMethod(tree);
    fbf.setStdDev(0.1);
    fbf.setHalfSize(0.1);
    fbf.filter(*output);
}

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    pcl::PCDReader reader;
    reader.read<PointT>(argv[1], *cloud);

    bilateralFilter(cloud, cloud_filtered);

    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
    // 显式指定 PointXYZI 类型
    // 错误代码：viewer.addPointCloud(cloud_filtered, "cloud_filtered");
    // 这是由于 bilateralFilter需要PointXYZI，而addPointCloud默认使用PointXYZ
    viewer.addPointCloud<PointT>(cloud_filtered, "cloud_filtered");
    viewer.spin();

    return 0;
}
