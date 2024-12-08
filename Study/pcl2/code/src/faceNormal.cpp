#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT; // 使用 PointNormal，这样可以显示法向量

int main(int argc, char** argv)
{
    // 检查是否提供了PCD文件路径作为命令行参数
    if (argc < 2) {
        std::cerr << "请提供PCD文件的路径作为参数。" << std::endl;
        return -1;
    }

    // 创建一个PointCloud对象，用于存储点云数据
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    
    // 加载PCD文件，如果加载失败则输出错误信息
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
        std::cerr << "无法读取文件 " << argv[1] << std::endl;
        return -1;
    }

    // 计算法向量
    pcl::NormalEstimation<PointT, PointNT> nest;
    nest.setInputCloud(cloud);
    nest.setKSearch(50); // 设置计算法向量时使用的邻居点数
    pcl::PointCloud<PointNT>::Ptr cloud_normals(new pcl::PointCloud<PointNT>);
    nest.compute(*cloud_normals);

    // 将原始点云数据复制到法向量点云中，这样可以用于可视化
    for (size_t i = 0; i < cloud_normals->points.size(); i++) {
        cloud_normals->points[i].x = cloud->points[i].x;
        cloud_normals->points[i].y = cloud->points[i].y;
        cloud_normals->points[i].z = cloud->points[i].z;
    }

    // 创建一个PCL可视化器对象
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

    // 将原始点云添加到可视化器中
    viewer.addPointCloud<PointT>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    // 将包含法向量的点云添加到可视化器中
    viewer.addPointCloud<PointNT>(cloud_normals, "cloud_normals");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_normals");

    // 在可视化器中显示法向量，设置法向量的长度和显示的点数
    int level = 100; // 每个法向量显示时显示多少个点
    float scale = 1; // 法向量的长度
    viewer.addPointCloudNormals<PointNT>(cloud_normals, level, scale, "cloud_normals_with_normals");

    // 启动可视化器并开始可视化
    viewer.spin();

    return 0;
}
