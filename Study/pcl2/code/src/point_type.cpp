#include<iostream>
#include<pcl/point_types.h>

int main(int argc, char** argv)
{
    pcl::PointXYZRGB point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.r = 255;
    point.g = 0;
    point.b = 0;
    std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

    pcl::PointXYZRGBNormal point_normal;
    point_normal.x = 1.0f;
    point_normal.y = 2.0f;
    point_normal.z = 3.0f;
    point_normal.normal_x = 0.0f;
    point_normal.normal_y = 0.0f;
    point_normal.normal_z = 1.0f;
    point_normal.curvature = 0.0f;

    std::cout << "Point Normal: (" << point_normal.x << ", " << point_normal.y << ", " << point_normal.z << ", " << point_normal.normal_x << ", " << point_normal.normal_y << ", " << point_normal.normal_z << ", " << point_normal.curvature << ")" << std::endl;
    return 0;

}