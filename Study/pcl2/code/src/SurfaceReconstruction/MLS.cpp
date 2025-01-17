#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "resolution.h"
int main(int argc, char **argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile(argv[1], *cloud);
    double resolution = computeCloudResolution(cloud);
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search ::KdTree<pcl::PointXYZ>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setInputCloud(cloud);
    mls.setComputeNormals(true);    // 我们都知道表面重构时需要估计点云的法向量，这里MLS提供了一种方法来估计点云法向量。（如果是true的话注意输出数据格式）。
    //mls.setPolynomialFit(true);     // 对于法线的估计是有多项式还是仅仅依靠切线。
    mls.setPolynomialOrder(3); // MLS拟合曲线的阶数，这个阶数在构造函数里默认是2，但是参考文献给出最好选择3或者4
    mls.setSearchMethod(tree);
    mls.setSearchRadius(10 * resolution);
    // Reconstruct
    mls.process(mls_points);
    // Save output
    // pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
    // show
    pcl::visualization::PCLVisualizer viewer_1;
    viewer_1.addPointCloud(cloud, "cloud");
    viewer_1.spin();
    pcl::visualization::PCLVisualizer viewer_2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(mls_points, *result);
    viewer_2.addPointCloud(result, "mls_points");
    viewer_2.spin();
    return 0;
}