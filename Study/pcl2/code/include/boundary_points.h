#ifndef _BOUNDARY_POINTS_
#define _BOUNDARY_POINTS_
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
void computeBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &
                               cloud,
                           double resolution,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &boundary_cloud)
{
    // compute normals;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
    normal_est.setSearchMethod(tree);
    normal_est.setInputCloud(cloud);
    normal_est.setKSearch(50);
    normal_est.compute(*normals);
    // normal_est.setViewPoint(0,0,0);
    // calculate boundary;
    pcl::PointCloud<pcl::Boundary> boundary;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_est;
    boundary_est.setInputCloud(cloud);
    boundary_est.setInputNormals(normals);
    boundary_est.setRadiusSearch(5 * resolution);
    boundary_est.setAngleThreshold(M_PI / 4);
    boundary_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    boundary_est.compute(boundary);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (boundary.points[i].boundary_point == 1)
            boundary_cloud->push_back(cloud->points[i]);
    }
    std::cout << "boundary size is " << boundary_cloud->points.size() << std::endl;
}
void eliminateBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &keys,
                       double resolution, int rate)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    computeBoundaryPoints(cloud, resolution, boundary_cloud);
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(boundary_cloud);
    std::vector<int> indices;
    std::vector<float> distances;
    float diff = rate * rate * resolution * resolution;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keys_result(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < keys->points.size(); ++i)
    {
        kdtree.nearestKSearch(keys->points[i], 1, indices, distances);
        if (distances[0] > diff)
            keys_result->push_back(keys->points[i]);
    }
    std::cout << "remove " << keys->points.size() - keys_result->points.size() << "points" << std::endl;
    keys->clear();
    keys = keys_result;
}
#endif // ! _BOUNDARY_POINTS_