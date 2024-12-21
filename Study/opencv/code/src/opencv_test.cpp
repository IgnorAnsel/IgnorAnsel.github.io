#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 加载图像
    cv::Mat image = cv::imread("image.jpg");  // 需要替换成自己的图像路径

    // 检查图像是否成功加载
    if (image.empty()) {
        std::cerr << "Error: Could not load image!" << std::endl;
        return -1;
    }

    // 显示原始图像
    cv::imshow("Original Image", image);

    // 转换为灰度图像
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    // 显示灰度图像
    cv::imshow("Gray Image", grayImage);

    // 等待按键并退出
    cv::waitKey(0);

    return 0;
}
