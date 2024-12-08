@[7.表面法线]()

# PCL 点云法向量计算与可视化

## 1.什么是表面法线

法向量（Normal Vector）是指垂直于一个表面的向量。它在三维空间中的应用非常广泛，特别是在计算机图形学和计算机视觉中。法向量通常用于表示一个表面的朝向或者平面上的切平面方向。

### 法向量的意义
- 几何学定义：法向量是与表面平行的切平面垂直的向量。简单来说，法向量垂直于点云的表面，用来描述表面局部的方向。
- 在点云中的作用：在三维点云中，每个点都有一个法向量，用来表示该点所在表面的朝向。法向量对于后续的处理（如表面重建、点云分割等）非常重要。

## 2.PCL 点云法向量计算与可视化代码详解

### 2.1完整代码

```cpp
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
        // 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
        // 也可用其他方法进行连接，如：pcl::concatenateFields
        cloud_normals->points[i].x = cloud->points[i].x;
        cloud_normals->points[i].y = cloud->points[i].y;
        cloud_normals->points[i].z = cloud->points[i].z;

        //pcl::concatenateFields(*cloud, *cloud_normals, *cloud_normals);
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

```

### 2.2代码详解

#### 2.2.1 头文件包含

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h> // 用于计算点云的法向量。
#include <pcl/visualization/pcl_visualizer.h>
```

这些头文件包含了处理点云所需的各种库和类。

#### 2.2.2 定义点云类型

```cpp
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT; // 使用 PointNormal，这样可以显示法向量

```
更加具体可见[3.PCL点云数据类型](.log/begin_pcl2/PointType.md)

#### 2.2.3 加载点云数据

```cpp
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1) {
    std::cerr << "无法读取文件 " << argv[1] << std::endl;
    return -1;
}
```

这段代码从命令行参数中获取PCD文件的路径，并加载点云数据到`cloud`对象中。如果文件加载失败，则输出错误信息并返回-1。

#### 2.2.4 计算法向量

```cpp
pcl::NormalEstimation<PointT, PointNT> nest;
nest.setInputCloud(cloud);
nest.setKSearch(50); // 设置计算法向量时使用的邻居点数
pcl::PointCloud<PointNT>::Ptr cloud_normals(new pcl::PointCloud<PointNT>);
nest.compute(*cloud_normals);
```
- 创建一个 pcl::NormalEstimation 对象 nest 来计算法向量。
- nest.setInputCloud(cloud) 设置输入的点云为 cloud。
- nest.setKSearch(50) 设置计算法向量时使用的邻域点数，即每个点的法向量计算会依赖于其周围的 50 个点。
- nest.compute(*cloud_normals) 计算法向量并将结果存储在 cloud_normals 中。

#### 2.2.5 可视化

```cpp
// 将原始点云数据复制到法向量点云中，这样可以用于可视化
for (size_t i = 0; i < cloud_normals->points.size(); i++) {
    // 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
        // 也可用其他方法进行连接，如：pcl::concatenateFields
        cloud_normals->points[i].x = cloud->points[i].x;
        cloud_normals->points[i].y = cloud->points[i].y;
        cloud_normals->points[i].z = cloud->points[i].z;

        //pcl::concatenateFields(*cloud, *cloud_normals, *cloud_normals);
}
```

这段代码将原始点云数据复制到法向量点云中，这样可以用于可视化。如果不进行这一步，只有法向量信息而没有原始点云信息，无法正确显示点云。

```cpp
pcl::visualization::PCLVisualizer viewer("法向量可视化");
viewer.setBackgroundColor(0.0, 0.0, 0.0); // 设置背景颜色为黑色
viewer.addPointCloud<PointT>(cloud, "cloud");
```
- 创建一个 pcl::visualization::PCLVisualizer 对象 viewer 来显示点云和法向量。
- viewer.setBackgroundColor(0.0, 0.0, 0.0) 设置可视化器的背景颜色为黑色。
- viewer.addPointCloud<PointT>(cloud, "cloud") 将原始点云数据添加到可视化器中。

```cpp
int level = 100; // 每个法向量显示时显示多少个点
float scale = 1; // 法向量的长度
viewer.addPointCloudNormals<PointNT>(cloud_normals, level, scale, "cloud_normals_with_normals");
```
- viewer.addPointCloudNormals<PointNT>(cloud_normals, level, scale, "cloud_normals_with_normals") 将法向量添加到可视化器中。参数 level 表示每个法向量显示多少个点，scale 表示法向量的长度。

```cpp
viewer.spin();
//while (!viewer.wasStopped()) {
//    viewer.spinOnce();
//}
```




