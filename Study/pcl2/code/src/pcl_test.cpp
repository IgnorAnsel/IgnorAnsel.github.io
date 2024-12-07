#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <random>

int main()
{
    // 创建一个点云对象，使用带有强度信息的 PointXYZI 类型
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 设置点云的尺寸
    int num_points = 1000; // 生成1000个点
    cloud->width = num_points;
    cloud->height = 1; // 单行点云（每个点都是一个独立的元素）

    cloud->points.resize(cloud->width * cloud->height);

    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10.0f, 10.0f); // 随机生成范围[-10, 10]
    std::uniform_real_distribution<float> intensity_dis(0.0f, 255.0f); // 随机生成强度值，范围[0, 255]

    // 随机生成点及其强度值
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = dis(gen);
        cloud->points[i].y = dis(gen);
        cloud->points[i].z = dis(gen);
        cloud->points[i].intensity = intensity_dis(gen); // 给每个点随机赋予一个强度值
    }

    std::cout << "Generated " << cloud->width * cloud->height << " random points with intensity." << std::endl;
    // 保存生成的点云
    pcl::io::savePCDFileASCII("random_cloud.pcd", *cloud);
    // 创建一个体素网格滤波器对象，进行降采样
    pcl::VoxelGrid<pcl::PointXYZI> sor; // 使用带强度信息的滤波器
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);  // 设置体素大小
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    sor.filter(*cloud_filtered);  // 对点云进行滤波

    std::cout << "PointCloud after filtering has: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // 保存处理后的点云
    pcl::io::savePCDFileASCII("random_filtered_cloud.pcd", *cloud_filtered);
    std::cout << "Saved filtered point cloud to 'random_filtered_cloud.pcd'" << std::endl;

    // 可视化生成的点云
    pcl::visualization::CloudViewer viewer("Random Cloud Viewer");
    viewer.showCloud(cloud_filtered);

    // 阻塞方式等待直到用户关闭窗口
    while (!viewer.wasStopped())
    {
        // 阻塞显示点云
        pcl_sleep(0.1);  // 使用 pcl_sleep 可以避免过高的 CPU 占用
    }

    return 0;
}
