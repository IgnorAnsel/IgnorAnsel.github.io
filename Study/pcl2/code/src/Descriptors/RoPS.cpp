#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/features/rops_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/keypoints/iss_3d.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NT;
typedef pcl::PointNormal PointNT;
typedef pcl::Histogram<135> FeatureT;
pcl::PointCloud<PointT>::Ptr GetKeyPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    double resolution = computeCloudResolution(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::ISSKeypoint3D<PointT, PointT> iss;
    iss.setSearchMethod(tree);
    iss.setSalientRadius(6 * resolution);
    iss.setNonMaxRadius(4 * resolution);
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.setNumberOfThreads(4);
    iss.setInputCloud(cloud);

    iss.compute(*key_cloud);
    return key_cloud;
}
int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr key_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointNT>::Ptr cloud_normals(new pcl::PointCloud<PointNT>);

    if (pcl::io::loadPLYFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return (-1);
    }
    double resolution = computeCloudResolution(cloud);
    key_cloud = GetKeyPointCloud(cloud);
    pcl::NormalEstimation<PointT, PointNT> ne;
    ne.setKSearch(10);
    ne.setInputCloud(cloud);
    ne.setSearchSurface(cloud);
    ne.compute(*cloud_normals);
    std::cout << "compute normal\n";

    // 连接数据
    pcl::PointCloud<PointNT>::Ptr key_cloud_normals(new pcl::PointCloud<PointNT>);
    pcl::concatenateFields(*cloud, *cloud_normals, *key_cloud_normals);

    // ---- rops基于网格，所以要先将pcd点云数据重建网格 ---
    pcl::search::KdTree<PointNT>::Ptr tree(new pcl::search::KdTree<PointNT>);
    tree->setInputCloud(key_cloud_normals);
    pcl::GreedyProjectionTriangulation<PointNT> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(key_cloud_normals);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(triangles);

    // rops
    pcl::ROPSEstimation<PointT,FeatureT> rops;
    rops.setInputCloud(key_cloud);
    rops.setSearchSurface(cloud);
    rops.setNumberOfPartitionBins(5);
    rops.setNumberOfRotations(3);
    rops.setRadiusSearch(0.01);
    rops.setSupportRadius(0.01);
    rops.setTriangles(triangles.polygons);
    rops.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree < pcl::PointXYZ>));
    //feature size = number_of_rotations * number_of_axis_to_rotate_around * number_of_projections * number_of_central_moments
    //unsigned int feature_size = number_of_rotations_ * 3 * 3 * 5; //计算出135
    pcl::PointCloud<FeatureT> description;
    rops.compute(description);
    std::cout << "size is " << description.points.size()<<std::endl;
    std::cout << "compute rops\n";
    return 0;

}