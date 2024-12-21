#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <string>
typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cout << "Usage: ./Rotate_pcl input.pcd" << std::endl;
        return -1;
    }

    if(argv[1][strlen(argv[1])-1] == 'd') // pcd
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile<PointT>(argv[1], *cloud);

        // 随机生成旋转轴（单位向量）
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);

        Eigen::Vector3f axis(dis(gen), dis(gen), dis(gen));
        axis.normalize();  // 归一化为单位向量

        // 随机生成旋转角度（0到360度之间）
        std::uniform_real_distribution<> angle_dis(0.0, 360.0);
        float angle = angle_dis(gen);
        // 将角度转换为弧度
        float angle_in_radians = angle * M_PI / 180.0f;
        // 创建旋转矩阵
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix = Eigen::AngleAxisf(angle_in_radians, axis);

        // 应用旋转到点云
        for(auto& point : *cloud)
        {
            Eigen::Vector3f point_vec(point.x, point.y, point.z);
            point_vec = rotation_matrix * point_vec;
            point.x = point_vec[0];
            point.y = point_vec[1];
            point.z = point_vec[2];
        }
        std::string fileName = argv[1];
        fileName = "output_" + fileName;
        // 保存旋转后的点云
        pcl::io::savePCDFileASCII(fileName, *cloud);

        std::cout << "Point cloud rotated and saved to 'rotated_output.pcd'" << std::endl;
    }
    else if(argv[1][strlen(argv[1])-1] == 'y') // ply
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::io::loadPLYFile<PointT>(argv[1], *cloud);

        // 随机生成旋转轴（单位向量）
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);

        Eigen::Vector3f axis(dis(gen), dis(gen), dis(gen));
        axis.normalize();  // 归一化为单位向量

        // 随机生成旋转角度（0到360度之间）
        std::uniform_real_distribution<> angle_dis(0.0, 360.0);
        float angle = angle_dis(gen);
        // 将角度转换为弧度
        float angle_in_radians = angle * M_PI / 180.0f;
        // 创建旋转矩阵
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix = Eigen::AngleAxisf(angle_in_radians, axis);

        // 应用旋转到点云
        for(auto& point : *cloud)
        {
            Eigen::Vector3f point_vec(point.x, point.y, point.z);
            point_vec = rotation_matrix * point_vec;
            point.x = point_vec[0];
            point.y = point_vec[1];
            point.z = point_vec[2];
        }
        std::string fileName = argv[1];
        fileName = "output_" + fileName;
        // 保存旋转后的点云
        pcl::io::savePLYFileASCII(fileName, *cloud);

        std::cout << "Point cloud rotated and saved to 'rotated_output.pcd'" << std::endl;
    }

    return 0;
}
