@[TOC](PCL2安装)

## VTK安装
- 介绍：VTK是一个开源的、跨平台的、用于3D计算机图形、图像处理和可视化的开源软件系统。VTK是Visualization Toolkit的简称，它由一系列C++类构成，用于交互式地2D和3D科学可视化。VTK利用对象导向的技术以及松耦合的设计原则，使开发人员能够利用VTK丰富的算法类库开发自己的高级可视化应用。
- 下载链接：[8.2.0版本](https://vtk.org/files/release/8.2/VTK-8.2.0.zip) 其他版本请自行搜索
- 安装步骤：
 1. 安装依赖库：
    ```bash
    sudo apt-get install cmake-curses-gui
    sudo apt-get install freeglut3-dev
    # X11
    sudo apt-get install libx11-dev libxext-dev libxtst-dev libxrender-dev libxmu-dev libxmuu-dev
    # OpenGL
    sudo apt-get install build-essential libgl1-mesa-dev libglu1-mesa-dev
    # cmake && cmake-gui
    sudo apt-get install cmake cmake-gui
    ```
 2. 解压下载的文件，进入解压后的文件夹，打开命令行，输入以下命令：
     ```bash
     mkdir build
     cd build
     cmake .. -DCMAKE_BUILD_TYPE=Release
     然后退出到安装目录的上一级的build内，输入cmake-gui，如下配置：
     BUILD_SHARED_LIBS=ON   
 
     BUILD_TESTING=OFF # 默认OFF，如果打开的话，编译时会由于下载测试数据所用url过旧而报错，建议OFF
 
     CMAKE_BUILD_TYPE=Release    # 默认Debug运行会较慢
 
     CMAKE_INSTALL_PREFIX=/usr/local    # 这里用默认就行，或者改到想要安装的位置
 
     VTK_FORBID_DOWNLOADS=ON    # 默认OFF，建议打开，否则编译会报错，理由同BUILD_TESTING
     VTK_USE_SYSTEM_PNG =ON
     配置生成后退回原来的build
     make -j4 #后面的数字代表线程数，根据电脑配置自行调整
     #如果编译卡住了Ctrl+C终止，重新make
     sudo make install
     ```
 3. 如果中途出现报错，请根据报错信息进行解决，例如缺少某个库，就安装对应的库，例如缺少freeglut3-dev，就安装freeglut3-dev。以下有推荐的安装方式1：
    ```bash
    git clone https://github.com/microsoft/vcpkg.git
    cd vcpkg
    ./bootstrap-vcpkg.sh
    使用vcpkg安装依赖库，方便，就是看你网了
    ./vcpkg install cmake qt5 libpng libjpeg libtiff fftw eigen #安装依赖库

    ```
    方式2：
    ```bash
    sudo apt install cmake g++ libxt-dev libxi-dev libxmu-dev libgl1-mesa-dev \
                 libpng-dev libjpeg-dev libtiff-dev \
                 libfftw3-dev qt5-qmake qtbase5-dev libsqlite3-dev \
                 libeigen3-dev
    ```
 4. 可能出现问题的解决方法：
    - 如果出现`multiple definition of ‘exodus_unused_symbol_dummy_1’`：
      ```bash
      找到位置然后注释掉
      ```
    - 待补充问题
    
## PCL2安装
- 介绍：PCL2是Point Cloud Library 2.0的简称，它是一个开源的、跨平台的、用于3D点云处理的开源软件库。PCL2提供了丰富的算法和工具，用于点云数据的处理、分析和可视化。PCL2支持多种操作系统，包括Windows、Linux和Mac OS X。PCL2还提供了丰富的文档和示例代码，帮助开发人员快速上手和使用PCL2。
- 下载链接：[PCL 最新版本](https://objects.githubusercontent.com/github-production-release-asset-2e65be/8162615/6f1184f6-6ac2-4ae2-b884-4bd39c2d4352?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=releaseassetproduction%2F20241206%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20241206T103346Z&X-Amz-Expires=300&X-Amz-Signature=a281ea5a54d28781c1964e601e72a8525edb85aebb2245f320817d838a41796d&X-Amz-SignedHeaders=host&response-content-disposition=attachment%3B%20filename%3Dsource.zip&response-content-type=application%2Foctet-stream)
或者
    ```bash
    git clone https://github.com/PointCloudLibrary/pcl.git获取最新
    ```
- 安装步骤：
  1. 安装依赖库：
     ```bash
     sudo apt-get update
     sudo apt-get install git build-essential linux-libc-dev 
     sudo apt-get install cmake cmake-gui
     sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
     sudo apt-get install mpi-default-dev openmpi-bin openmpi-common
     sudo apt-get install libflann1.9 libflann-dev  
     sudo apt-get install libeigen3-dev
     sudo apt-get install libboost-all-dev
     sudo apt-get install libqhull* libgtest-dev
     sudo apt-get install freeglut3-dev pkg-config
     sudo apt-get install libxmu-dev libxi-dev
     sudo apt-get install mono-complete
     sudo apt-get install libopenni-dev
     sudo apt-get install libopenni2-dev
     ```
  2. 安装PCL2：
    解压下载的文件，进入解压后的文件夹，打开命令行，输入以下命令：
     ```bash
     mkdir build
     cd build
     cmake ..
     make
     make install
     ```
- 偷懒的安装方式：
    ```bash
    sudo apt install libpcl-dev
    ```

 
