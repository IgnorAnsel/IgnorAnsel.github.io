#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "resolution.h"

int main(int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(argv[1], cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    std::cout << "Loaded cloud with " << cloud->points.size() << " points." << std::endl;

    double resolution = computeCloudResolution(cloud);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    // normals should not contain the point normals + surface curvatures
    std::cout << "Number of normals: " << normals->points.size() << std::endl;

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    // cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>());
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::MarchingCubes<pcl::PointNormal>::Ptr mc(new pcl::MarchingCubesHoppe<pcl::PointNormal>);
    //设置MarchingCubes对象的参数
    mc->setIsoLevel(0.0f);
    mc->setGridResolution(5, 5, 5);
    mc->setPercentageExtendGrid(0.0f);
    mc->setSearchMethod(tree2);
    mc->setInputCloud(cloud_with_normals);
    pcl::PolygonMesh mesh;//执行重构，结果保存在mesh中
    mc->reconstruct(mesh);

    // Visualization
    pcl::visualization::PCLVisualizer viewer("MarchingCubes");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPolygonMesh(mesh, "meshs");
    viewer.spin();
    return 0;
    
}