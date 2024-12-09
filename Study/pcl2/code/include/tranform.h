#pragma once
#include <pcl/common/transforms.h>
void MyTransformationPoint(pcl::PointXYZ &pt_in, pcl::PointXYZ &pt_out, float tx, float ty, float tz, float rx, float ry, float rz)
{
    ////rotating and translating point cloud
    ////m and m1 are both the unit matrixes
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
    // tx,ty,tz are translation values of x ,y and z axis
    m1(0, 3) = tx;
    m1(1, 3) = ty;
    m1(2, 3) = tz;
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the translation
    // cout << "m" << endl << m << endl;
    // rotating the point cloud along the x axis
    m1 = Eigen::Matrix4f::Identity();
    rx = rx / 180.0 * M_PI;
    m1(1, 1) = cos(rx);
    m1(1, 2) = -sin(rx);
    m1(2, 1) = sin(rx);
    m1(2, 2) = cos(rx);
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the rotation along x axis
    // cout << "m" << endl << m << endl;
    // rotating the point cloud along the y axis
    m1 = Eigen::Matrix4f::Identity();
    ry = ry / 180.0 * M_PI;
    m1(0, 0) = cos(ry);
    m1(0, 2) = sin(ry);
    m1(2, 0) = -sin(ry);
    m1(2, 2) = cos(ry);
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the rotation along y axis
    // cout << "m" << endl << m << endl;
    // rotating the point cloud along the z axis
    m1 = Eigen::Matrix4f::Identity();
    rz = rz / 180.0 * M_PI;
    m1(0, 0) = cos(rz);
    m1(0, 1) = -sin(rz);
    m1(1, 0) = sin(rz);
    m1(1, 1) = cos(rz);
    // cout << "m1" << endl << m1 << endl;
    m = m * m1;
    // apply the rotation along y axis
    // cout << "m" << endl << m << endl;
    // pcl::transformPointCloud(src, dst, m);
    Eigen::Matrix<float, 3, 1> pt(pt_in.x, pt_in.y, pt_in.z);
    pt_out.x = static_cast<float>(m(0, 0) * pt.coeffRef(0) + m(0, 1) * pt.coeffRef(1) + m(0, 2) * pt.coeffRef(2) + m(0, 3));
    pt_out.y = static_cast<float>(m(1, 0) * pt.coeffRef(0) + m(1, 1) * pt.coeffRef(1) + m(1, 2) * pt.coeffRef(2) + m(1, 3));
    pt_out.z = static_cast<float>(m(2, 0) * pt.coeffRef(0) + m(2, 1) * pt.coeffRef(1) + m(2, 2) * pt.coeffRef(2) + m(2, 3));
}