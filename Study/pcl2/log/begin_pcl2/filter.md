@[6.点云滤波]()

PCL中总结了集中需要进行点云滤波处理的情况，分别如下： (1) 点云数据密度不
规则需要平滑。 (2) 因为遮挡等问题造成离群点需要去除。 (3) 大量数据需要进行
下采样。 (4) 噪声数据需要去除。
PCL点云格式分为有序点云和无序点云，针对有序点云提供了双边滤波、高斯滤
波、中值滤波等，针对无序点云提供了体素栅格、随机采样等。
下边给出两个下采样类的应用实例。（VoxelGrid、UniformSampling）

## 下采样

### VoxelGrid

VoxelGrid类通过输入的点云数据创建一个三维体素栅格，然后对每个体素栅格
内的点进行下采样。体素栅格内所有点用体素栅格的中心点表示，从而实现了点云
的下采样。VoxelGrid类的实现如下：

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file bun0.pcd \n");
        return (-1);
    }

    // 使用体素栅格滤波器进行降采样
    pcl::VoxelGrid<PointT> vg; // 创建滤波器对象
    vg.setInputCloud(cloud);  // 设置输入点云
    const float leaf = 0.05f; // 设置体素栅格的尺寸
    vg.setLeafSize(leaf, leaf, leaf); // 设置体素栅格的体积
    vg.filter(*cloud_filtered); // 执行滤波操作
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;
    return 0;
}
```

### UniformSampling

UniformSampling类通过随机下采样点云数据，从而实现点云数据的下采样。UniformSampling类的实现如下：

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file bun0.pcd \n");
        return (-1);
    }

    // 使用UniformSampling滤波器
    pcl::UniformSampling<PointT> us ; // 创建滤波器对象
    us.setInputCloud(cloud);  // 设置输入点云
    double radius = 0.01; // 设置体素栅格的半径
    us.setRadiusSearch(radius); // 设置搜索半径
    us.filter(*cloud_filtered); // 执行滤波操作
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;
    return 0;
}
```

## 双边滤波

双边滤波器是一种非线性滤波器，它通过考虑像素点之间的空间距离和颜色相似度来
对图像进行滤波。双边滤波器在保留边缘的同时，可以有效地去除噪声。双边滤波PCL提供了两种方法，FastBilateralFilter 和 BilateralFilter。
FastBilateralFilter需要有序点云数据，BilateralFilter需要带强度的点云数据。

```cpp
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
```