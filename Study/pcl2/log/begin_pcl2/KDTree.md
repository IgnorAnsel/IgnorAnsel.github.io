@[4.KDTree](这里写自定义目录标题)

# KDTree

## 1. KDTree简介

KDTree（k-dimensional tree）是一种用于多维空间中快速检索的树形数据结构。它是一种二叉搜索树，其中每个节点表示一个k维空间中的点，并且每个节点将空间划分为两个子空间。KDTree的主要用途是进行最近邻搜索，即在一个多维空间中找到距离给定点最近的点。

KDTree的构建过程如下：

1. 选择一个维度，将空间划分为两个子空间，使得每个子空间中的点尽可能均匀分布。这可以通过计算每个维度的方差来实现。
2. 选择方差最大的维度作为划分维度，将空间划分为两个子空间。
3. 对每个子空间递归地重复步骤1和步骤2，直到达到预定的深度或子空间中的点数小于预定的阈值。
4. 在每个节点中存储一个点，以及指向子节点的指针。

KDTree的查询过程如下：

1. 从根节点开始，根据查询点的维度选择一个子空间，递归地搜索该子空间。
2. 如果查询点在子空间中，则递归地搜索另一个子空间。
3. 如果查询点不在子空间中，则递归地搜索另一个子空间。
4. 当达到叶子节点时，返回该节点中存储的点作为最近邻点。
5. 在搜索过程中，可以记录下当前最近邻点及其距离，并在搜索过程中更新最近邻点及其距离。

KDTree的优点是它可以在O(log n)的时间复杂度内找到最近邻点，其中n是空间中的点数。这使得KDTree在处理大规模数据集时非常高效。

KDTree的缺点是它需要大量的内存来存储树结构，并且在构建过程中需要计算每个维度的方差，这可能会增加计算时间。

## 2. KDTree在PCL中的应用
 ### 2.1 KDTree在PCL中的实现
 PCL中类 pcl::KdTree<PointT> 是kd-tree数据结构的实现。并且提供基于
FLANN进行快速搜索的一些相关子类与包装类。具体可以参考相应的API。
 ### 2.2 pcl::search::KdTree < PointT >
 pcl::search::KdTree<PointT> 是 pcl::search::Search< PointT > 的子
类，是 pcl::KdTree<PointT> 的包装类。包含(1) k 近邻搜索；(2) 邻域半径搜
索。
 ```C++
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
typedef pcl::PointXYZ PointT;
int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("random_filtered_cloud.pcd", *cloud);

    // 创建一个KdTree对象
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud); // 设置要搜索的点云，建立KDTree

    std::vector<int> indices; // 存储查询近邻点索引
    std::vector<float> distances; // 存储近邻点对应距离的平方
    PointT point = cloud->points[0]; // 初始化一个查询点

    kdtree->nearestKSearch(point, 10, indices, distances); // 查询最近邻点
    std::cout << "Nearest 10 points:" << std::endl;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        std::cout << "Point: " << cloud->points[indices[i]].x << " "
                  << cloud->points[indices[i]].y << " "
                  << cloud->points[indices[i]].z << std::endl;
        std::cout << "Distance: " << distances[i] << std::endl;
    }

    double radius = 0.5;

    kdtree->radiusSearch(point, radius, indices, distances); // 查询半径为0.5的近邻点
    std::cout << "Points within radius " << radius << ":" << std::endl;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        std::cout << "Point: " << cloud->points[indices[i]].x << " "
                  << cloud->points[indices[i]].y << " "
                  << cloud->points[indices[i]].z << std::endl;
        std::cout << "Distance: " << distances[i] << std::endl;
    }
    return 0;
}
 ```
 注意： 搜索结果默认是按照距离point点的距离从近到远排序；如果InputCloud中含
有point点，搜索结果的的第一个点是point本身。
 ### 2.3 pcl::KdTreeFLANN < PointT >
 pcl::KdTreeFLANN<PointT> 是 pcl::KdTree<PointT> 的包装类，是FLANN（Fast
Library for Approximate Nearest Neighbors）的包装类。包含(1) k 近邻搜索；(2) 邻域半径搜索。
 ```C++
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/flann_search.h> // 注意：需要包含这个头文件，在1.14版本是这个头文件，之前的一些版本可能需要改为：#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile<PointT>(argv[1], *cloud);

    pcl::search::FlannSearch<PointT> kdtree; // 创建一个KdTreeFLANN对象 注意：在1.14版本是FlannSearch，之前的一些版本可能需要改为：pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree.nearestKSearch(cloud->points[0], 10, indices, distances);
    std::cout << "Nearest 2 points to the first point: " << std::endl;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        std::cout << "Index: " << indices[i] << ", Distance: " << distances[i] << std::endl;
    }
    return 0;
}
```
