#include <iostream>
#include <OpenNI.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace openni;

int main() {
    // 初始化 OpenNI
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        cerr << "OpenNI initialization failed: " << OpenNI::getExtendedError() << endl;
        return 1;
    }
    // 获取设备信息
    Array<DeviceInfo> deviceInfos;
    OpenNI::enumerateDevices(&deviceInfos);  // 传入设备信息数组


    // 输出所有设备的 URI
    for (int i = 0; i < deviceInfos.getSize(); ++i) {
        const DeviceInfo& deviceInfo = deviceInfos[i];
        cout << "Device " << i << ": " << deviceInfo.getUri() << endl;
    }

    // 打开设备
    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK) {
        cerr << "Failed to open device: " << OpenNI::getExtendedError() << endl;
        return 1;
    }

    // 获取深度流
    VideoStream depthStream;
    rc = depthStream.create(device, SENSOR_DEPTH);
    if (rc != STATUS_OK) {
        cerr << "Failed to create depth stream: " << OpenNI::getExtendedError() << endl;
        return 1;
    }

    // 启动深度流
    rc = depthStream.start();
    if (rc != STATUS_OK) {
        cerr << "Failed to start depth stream: " << OpenNI::getExtendedError() << endl;
        return 1;
    }

    // 获取相机内参（焦距和主点）
    VideoMode depthMode = depthStream.getVideoMode();
    int width = depthMode.getResolutionX();
    int height = depthMode.getResolutionY();

    // 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建 PCL 可视化对象
    pcl::visualization::CloudViewer viewer("PCL Viewer");

    // 获取深度数据并转换为 PCL 点云
    while (!viewer.wasStopped()) {
        // 获取深度帧
        VideoFrameRef frame;
        rc = depthStream.readFrame(&frame);
        if (rc != STATUS_OK) {
            cerr << "Failed to read frame: " << OpenNI::getExtendedError() << endl;
            break;
        }

        // 获取深度数据指针
        const DepthPixel* depthData = (const DepthPixel*)frame.getData();

        // 清空 PCL 点云
        cloud->clear();

        // 将深度数据转换为 PCL 点云
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                pcl::PointXYZ point;

                // 获取深度值（单位：毫米）
                float depth = (float)depthData[index];  // depth in mm

                // 将图像坐标（x, y）转换为 3D 空间坐标
                if (depth != 0) {  // 如果深度值有效（非0）
                    // 将像素坐标（x, y）转换为世界坐标系中的 3D 点
                    point.x = (x - width / 2) * depth / 1000.0f;  // 转换为米
                    point.y = (y - height / 2) * depth / 1000.0f;
                    point.z = depth / 1000.0f;  // 深度值转换为米
                    cloud->points.push_back(point);
                }
            }
        }

        // 显示点云
        viewer.showCloud(cloud);
    }

    // 关闭设备
    depthStream.stop();
    device.close();
    OpenNI::shutdown();

    return 0;
}
