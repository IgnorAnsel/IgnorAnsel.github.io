@[8.提取关键点](这里写自定义目录标题)

# 提取关键点

## 1.ISSKeyPoint3D

### 1.1 简介

ISS（Intrinsic Shape Signatures）是一种用于三维点云中提取关键点的算法。它通过计算点云中每个点的局部曲率信息，来识别出具有显著局部特征的关键点。ISS算法的主要优点是它能够有效地处理噪声和复杂形状的点云数据，并且具有较高的鲁棒性。

### 1.2 参数

- `salient_radius`：用于计算关键点的“显著性”范围。即在此范围内的点被视为局部区域，显著的点在该范围内更加突出。值越大，检测到的关键点越少。
- `non_max_radius`：用于非极大值抑制（NMS），即筛选出邻域范围内显著性最强的点，抑制不重要的点。控制是否允许出现多个局部最大值。较大的值可能导致更多的关键点被保留。
- `gamma_21`：该参数影响到显著性函数的计算，控制显著性特征的影响程度。值越大，对点的显著性影响越强。
- `gamma_32`：与 gamma_21 类似，控制第二部分的显著性计算。
- `min_neighbors`：用于控制在进行非极大值抑制时，关键点邻域中的最小邻居数。用于控制检测点的数量，较小的值会导致更多的关键点。
- `threshold21`：一个阈值，用于设置显著性计算中的两个参数之间的相对比例，通常与 salient_radius 和 gamma_21 配合使用，影响关键点的选择。
- `threshold32`：用于计算每个点的局部曲率信息的参数。
- `edge_threshold`：用于控制边缘效应检测的阈值。较小的值将更加注重边缘，而较大的值则会忽略边缘区域。
- `radius_search`：在计算关键点的邻域时，定义搜索的半径。用于定义点云数据中关键点的“密集”区域，影响检索时的范围。
- `use_radius_search`：设置是否使用基于半径的搜索来查找邻域点。启用此选项时，算法会根据设定的半径搜索邻域。

### 1.3 代码

```cpp
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/keypoints/iss_3d.h>

#include "resolution.h" // 用于计算点云分辨率 (见附录)

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    // 加载点云数据
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPLYFile<PointT>(argv[1], *cloud);
    std::cout << "origin cloud size: " << cloud->size() << std::endl;
    // 计算点云分辨率
    float resolution = computeCloudResolution(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::ISSKeypoint3D<PointT, PointT> iss; // 创建一个 ISSKeypoint3D 对象，PointT 为输入点和输出点的类型。
    iss.setSearchMethod(tree); // 设置搜索方法为 KdTree
    iss.setSalientRadius(6 * resolution); // 设置“显著性半径”参数，决定了计算每个点显著性的邻域大小。通常使用点云的分辨率来设置。
    iss.setNonMaxRadius(4 * resolution); // 设置非最大抑制的半径，影响最终选择哪些点作为关键点。
    iss.setThreshold21(0.975); // 设置阈值，用于显著性计算中的比例。
    iss.setThreshold32(0.975); // 与 threshold21 类似，用于第二部分的显著性计算。
    iss.setMinNeighbors(5); // 设置在进行非最大值抑制时，邻域内的最小邻居数量。较小的值会保留更多关键点。
    iss.setNumberOfThreads(4); // 设置用于并行计算的线程数。
    iss.setInputCloud(cloud); // 设置输入的点云数据。

    pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>);
    iss.compute(*keypoints); // 来执行关键点检测算法。检测到的关键点会被存储在 keypoints 中。
    std::cout << "keypoints size: " << keypoints->size() << std::endl;

    return 0;
}
```

## 2.HarrisKeypoint3D

### 2.1 简介

Harris角点检测是一种用于二维图像中的角点检测算法。它通过计算图像中每个像素点的局部曲率信息，来识别出具有显著局部特征的关键点。Harris角点检测算法的主要优点是它能够有效地处理噪声和复杂形状的图像数据，并且具有较高的鲁棒性。

### 2.2 参数

- `threshold`：设置Harris角点检测中的阈值。这个值用来确定哪些点被认为是关键点，通常是角点的响应函数值。只有大于该阈值的点才会被认为是一个关键点。
- `radius`：设置搜索邻域的半径。用于计算每个点的响应函数，邻域内的点用于计算Harris矩阵。较大的值通常会导致更多的关键点。
- `non_max_suppression`：是否启用非最大抑制。在提取关键点时，如果一个点周围的点具有更大的响应值，则该点会被抑制。启用后，只有局部最大值才会被保留。
- `k`：Harris角点检测算法中的一个常数参数，通常为0.04。它与计算特征响应的矩阵有关。这个值影响角点响应的灵敏度，较高的值可能会导致较少的关键点。
- `edge_threshold`：用于边缘检测的阈值。该参数帮助检测点云中的边缘，避免在边缘处提取关键点。如果点云中的响应值低于此值，则被认为是边缘。
- `min_neighbors`：用于非最大抑制的参数。在非最大抑制过程中，该参数定义了每个关键点周围需要考虑的最小邻居数量。较小的值会保留更多关键点。
- `search_method`：设置搜索方法（例如 pcl::search::KdTree<PointT>），用来在邻域内查找点。它通常用于加速点云内邻域搜索。
- `use_normal`：是否使用点云的法线信息来计算Harris响应。启用该选项可以使角点检测更加稳定，尤其是在处理不规则的点云时。

### 2.3 代码

```cpp
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/keypoints/harris_3d.h>

#include "resolution.h"

typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPLYFile<PointT>(argv[1], *cloud);

    std::cout << "original cloud size: " << cloud->size() << std::endl;

    double resolution = computeCloudResolution(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> harris; // 创建一个 HarrisKeypoint3D 对象，输入点类型为 PointT（即 pcl::PointXYZ），输出点类型为 pcl::PointXYZI（带有强度信息的点）。
    harris.setNonMaxSupression(true); // 启用非最大抑制（Non-Maximum Suppression）。这意味着在检测过程中，如果邻域内有响应值更大的点，则当前点会被抑制，确保只保留局部极大值作为关键点。
    harris.setRadiusSearch(10 * resolution); // 设置邻域搜索半径，使用点云的分辨率来动态调整邻域的大小。这里的 10 * resolution 表示邻域的半径是分辨率的10倍。
    harris.setThreshold(1E-6); // 设置Harris响应的阈值。只有当点的Harris响应值大于该阈值时，它才会被认为是一个关键点。1E-6 是一个较小的值，意味着只有响应非常显著的点才会被选为关键点。
    harris.setSearchMethod(tree);
    harris.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    harris.compute(*keypoints);

    pcl::console::print_highlight("Detected %d points !\n", keypoints->size());
    return 0;
}
```