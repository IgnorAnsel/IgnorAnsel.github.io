@[12.曲面重建]

# 曲面重建

## 简介
曲面重建是点云处理中的一种重要技术，它可以将点云数据转换为三维曲面模型。曲面重建在许多应用中都有广泛的应用，例如机器人导航、虚拟现实、计算机辅助设计等。

在PCL中，提供了多种曲面重建算法，包括泊松重建、贪婪投影三角化（Greedy Projection Triangulation，GPT）、Alpha形状（Alpha Shapes）等。这些算法各有优缺点，适用于不同的应用场景。

## 贪婪投影三角化算法

### 简介
贪婪投影三角化（Greedy Projection Triangulation，GPT）是一种基于点云数据的三角化算法。它通过投影点云数据到二维平面上，然后在平面上进行三角化，最后将三角化结果投影回三维空间中，得到曲面模型。

### 具体代码
```cpp
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "resolution.h"

int main(int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(argv[1], cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);

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

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    // cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>());
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(3 * resolution);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    // Visualization
    pcl::visualization::PCLVisualizer viewer("GreedyProjectionTriangulation");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPolygonMesh(triangles, "mesh");
    viewer.spin();
    return 0;
    
}
```

### 代码解释

#### 1. 加载点云数据
``` cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCLPointCloud2 cloud_blob;
pcl::io::loadPCDFile(argv[1], cloud_blob);
pcl::fromPCLPointCloud2(cloud_blob, *cloud);
```
- fromPCLPointCloud2(cloud_blob, *cloud)：将读取的点云数据从 PCLPointCloud2 类型转换为 pcl::PointCloud<pcl::PointXYZ> 类型，赋值给 cloud 变量。
其实这里主要是介绍一下

#### 2. 计算点云分辨率

- 略

#### 3. 法线估计

- 略

#### 4. 合并点云和法线信息

- 略

#### 5. 创建搜索树

- 略

#### 6. 网格重建（GreedyProjectionTriangulation）
``` cpp
pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
pcl::PolygonMesh triangles;
gp3.setSearchRadius(3 * resolution);
gp3.setMu(2.5);
gp3.setMaximumNearestNeighbors(100);
gp3.setMaximumSurfaceAngle(M_PI / 4);
gp3.setMinimumAngle(M_PI / 18);
gp3.setMaximumAngle(2 * M_PI / 3);
gp3.setNormalConsistency(false);
gp3.setInputCloud(cloud_with_normals);
gp3.setSearchMethod(tree2);
gp3.reconstruct(triangles);
```
- GreedyProjectionTriangulation：这是一个用于从点云数据重建表面网格的算法。它通过在邻域内选择点并连接成三角形来重建网格。
- setSearchRadius(3 * resolution)：设置最大搜索半径，影响点之间连接的最大距离。这里使用了点云分辨率的 3 倍。
- setMu(2.5)：控制点之间的平滑度，影响网格的细节。
- setMaximumNearestNeighbors(100)：设置搜索时最大邻居数目，影响计算的复杂度。
- setMaximumSurfaceAngle(M_PI / 4)：设置最大表面角度，控制网格面片的最大倾斜度。
- setMinimumAngle(M_PI / 18) 和 setMaximumAngle(2 * M_PI / 3)：分别设置网格三角形的最小和最大角度。
- setNormalConsistency(false)：如果设置为 true，则要求网格法线的方向一致性。
- gp3.reconstruct(triangles)：执行重建操作，将结果存储在 triangles（一个 PolygonMesh 对象）中。

#### 7. 获取网格的顶点信息
``` cpp
std::vector<int> parts = gp3.getPartIDs();
std::vector<int> states = gp3.getPointStates();
```
- getPartIDs()：获取网格分块的 ID（如果有分块的话）。
- getPointStates()：获取每个点的状态信息，指示该点是否是重建过程中使用的有效点。

#### 8. 可视化
``` cpp
pcl::visualization::PCLVisualizer viewer("GreedyProjectionTriangulation");
viewer.setBackgroundColor(0.0, 0.0, 0.0);
viewer.addPolygonMesh(triangles, "mesh");
viewer.spin();
```
- addPolygonMesh(triangles, "mesh")：将网格数据 triangles 添加到可视化窗口。

## 泊松曲面重建

### 简介
泊松曲面重建是一种用于从点云数据重建表面网格的算法。它通过求解泊松方程来重建表面，具有较好的重建质量和较高的效率。

### 具体代码
``` cpp
// Initialize objects
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false);
    pn.setDegree(2);
    pn.setDepth(10);
    pn.setIsoDivide(8);
    pn.setManifold(false);
    pn.setOutputPolygons(false);
    pn.setSamplesPerNode(3.0);
    pn.setScale(2.0);
    pn.setSolverDivide(8);
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
    pcl::PolygonMesh mesh;
    pn.performReconstruction(mesh);
    // 其他部分和前面一样
``` 

### 代码解释

#### Poisson
- Poisson：这是一个用于泊松曲面重建的类。首先初始化
- setConfidence(false)：该设置控制是否使用点的 "置信度"。在 Poisson 重建中，每个点可以具有一个置信度值，这个值通常表示该点被信任的程度，值越高表示该点越可靠。false 表示不使用置信度。也就是说，所有点都假定具有相同的置信度，且算法会依赖点云的法线信息进行重建。
- setDegree(2)：设置泊松方程的多项式阶数。degree 控制重建过程中多项式的复杂性。degree=2 表示使用二次多项式来拟合重建曲面。通常，degree 越高，曲面重建的精度越高，但计算复杂度也越大。
- setDepth(10)：depth 参数控制网格的分辨率。较高的 depth 会生成更精细的网格，捕捉更多的细节。在 depth=10 时，重建过程生成的网格包含更多的顶点，计算量较大，可能需要更多的内存和时间。
- setIsoDivide(8)：IsoDivide 控制泊松算法的等值面分割数。这个参数影响算法的精度和速度。它决定了对计算区域的分割方式，值越大，细节越多，但计算量也越大。IsoDivide=8 表示较高的分割数，通常适用于高分辨率或精细的网格重建。
- setManifold(false)：这个参数控制是否强制生成 "流形" 网格。流形网格是一个没有自交的网格，适用于要求连续曲面的应用场景。false 表示不强制生成流形网格。如果输入数据本身有些不连续或缺失，Poisson 重建仍然会尽可能构建一个网格。如果你的点云数据较为复杂，或者存在很多噪声点，保留 false 可能有助于避免重建失败。
- setOutputPolygons(false)：这个参数控制是否输出多边形网格。false 表示不输出多边形。如果需要最终的三角网格结果来进行可视化或者其他后续处理，应该设置为 true。当前设置意味着不需要多边形输出。true 时，Poisson 重建会生成一个 pcl::PolygonMesh 对象，其中包含三角形的顶点和面片信息。
- setSamplesPerNode(3.0)：samplesPerNode 参数控制在每个节点中使用多少个样本（点云的点数）。这个参数与网格的精度直接相关，值越高，每个节点的样本数越多，生成的网格会越精细，但计算量和内存使用也会增加。
3.0 是一个比较常用的默认值，适合大多数点云数据。
- setScale(2.0)：scale 控制重建的尺度或放缩因子。这个参数主要影响重建的网格的大小和分辨率。
2.0 表示放大重建结果的尺度，通常可以通过调整此参数来适应点云数据的大小，特别是在数据稀疏或过于密集时。
- setSolverDivide(8)：solverDivide 设置了求解器的划分数，影响算法的精度和性能。
较高的值可以提高重建的精度，但计算代价也更高。8 是一个合理的默认值，适用于多数场景。
- pn.setInputCloud(cloud_with_normals)：设置输入的点云数据，这里使用的是带有法线的点云。
- pn.performReconstruction(mesh)：执行泊松重建，将结果存储在 mesh（一个 PolygonMesh 对象）中。

## 移动立方体



