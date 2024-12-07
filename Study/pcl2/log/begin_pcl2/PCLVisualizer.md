@[5.PCLVisualizer](#5pclvisualizer)

## PCLVisualizer

### 1.简介

PCLVisualizer是PCL中一个非常重要的可视化工具，它是一个基于Qt的CPP类，可以用于显示点云、网格、法线、颜色、深度图等数据。PCLVisualizer可以用于调试和可视化点云处理算法，也可以用于展示点云数据。

### 2.使用方法

#### 2.1.创建PCLVisualizer对象

```CPP
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer viewer("3D Viewer");
```

#### 2.2.添加点云数据

```CPP
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("cloud.pcd", *cloud);

viewer.addPointCloud(cloud, "cloud");
```

#### 2.3.设置颜色

```CPP
viewer.setBackgroundColor(0.0, 0.0, 0.0); // 设置背景颜色，默认为黑色
```

```CPP
// 设置点云颜色
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud"); // rgb
pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud, 255, 0, 0); // rgb 将点云设置颜色，默认白色
viewer.addPointCloud(cloud, red, "cloud");

```
#### 2.4.设置相机位置

```CPP
viewer.setCameraPosition(0, 0, -2, 0, -1, 0); // 设置相机位置和方向，默认为(0, 0, 0, 0, 0, 1)
```

#### 2.5.显示窗口（两种方式）

```CPP
while (!viewer.wasStopped()) // 非阻塞显示
{
    viewer.spinOnce();
    std::this_thread::sleep_for(std::chrono::microseconds(100000));  // Sleep for 100ms
    // 可添加其他操作
}
```

```CPP
viewer.spin(); // 阻塞显示
```
#### 2.6 设置点云大小

```CPP
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud"); // 设置点云大小，默认为1
```
#### 2.7.添加线段

```CPP
pcl::PointXYZ p1(0, 0, 0);
pcl::PointXYZ p2(1, 1, 1);
viewer.addLine(p1, p2, "line");
```
#### 2.8.添加框

```CPP
viewer.addCube(0, 1, 0, 1, 0, 1, 1, 1, 1, "cube");
```
#### 2.9.添加文本

```CPP
viewer.addText("Hello, PCL!", 10, 10, "text");
```
#### 2.10.添加图像

```CPP
cv::Mat image = cv::imread("image.jpg", cv::IMREAD_COLOR);
viewer.addImage(image, "image");
```
#### 更多略,还请参考官方文档或者自行探索

## 完整代码

```CPP
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <thread>  // For std::this_thread
#include <chrono>  // For std::chrono

// PCL可视化库
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PointT;
int main(int argc, char** argv)
{
    // 创建一个点云对象
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // 读取点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file bunny.pcd \n");
        return (-1);
    }
    // 设置点云颜色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud, 255, 0, 0); // rgb 将点云设置颜色，默认白色
    // 创建一个可视化对象
    pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
    viewer.setBackgroundColor(100.0, 100.0, 100.0); // 设置背景颜色(默认为黑色)
    // 添加点云数据到可视化对象
    // "cloud1" 为显示id，默认cloud,显示多个点云时用默认会报警告。
    viewer.addPointCloud(cloud, red, "cloud1");
    // 将两个点连线
    PointT temp1 = cloud->points[0];
    PointT temp2 = cloud->points[1];
    viewer.addLine(temp1, temp2, "line0");
    // 同样可以设置线的颜色，
    //viewer.addLine(temp1, temp2, 255，0，0， "line0");

    // 显示点云数据
    // 开始显示2种方法,任选其一
    // 1. 阻塞式
    viewer.spin();
    // 2. 非阻塞式
    while (!viewer.wasStopped())
    {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::microseconds(100000));  // Sleep for 100ms
    // 可添加其他操作
    }
    
}
```

# 超完整代码，可供学习参考
[来源链接(官方)](https://pcl.readthedocs.io/projects/tutorials/en/latest/pcl_visualizer.html#pcl-visualizer)
```CPP
#include<iostream>
#include<thread>
 
#include<pcl/common/common_headers.h>
#include<pcl/features/normal_3d.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/console/parse.h>
 
using namespace std::chrono_literals;
 
// ----------------------
// -----打印使用帮助-----
// ----------------------
 
void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualization example\n"
		<< "-r           RGB colour visualization example\n"
		<< "-c           Custom colour visualization example\n"
		<< "-n           Normals visualization example\n"
		<< "-a           Shapes visualization example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}
//简单显示PointXYZ点云,返回一个视窗的shared智能指针
pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	//打开3D视窗并添加点云
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//设置背景颜色为黑色
	viewer->setBackgroundColor(0, 0, 0);
 
	/**
	 * 为视窗添加点云数据，传入一个字符串ID以便在其他方法中识别这个点云数据
	 * 可以多次调用addPointCloud()函数以添加多个点云数据，每次都提供一个新的ID
	 * 如果想更新一个已存在的点云数据，你可以调用removePointCloud()，并传入点云的ID使它能够得到更新
	 * 在PCL1.1及以上版本中，提供了一个新的API方法：updatePointCloud（）
	 * 使用它就可以直接更新点云，而不用先调用removePointCloud()函数删除再添加
	 * adddPointCloud()函数有许多重载形式，其他形式可以被用于操作不同类型的点云，显示法线等等
	 */
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
 
	//改变渲染点的大小，可以使用此方法在视窗中控制任何点云的渲染方式
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	
	/**
	 * 查看复杂的点云常常会使我们失去方向感，为了保证对点云位置的正确判断，可以显示一个坐标系
	 * 它们是沿着X轴（红色），Y轴（绿色），Z轴（蓝色）的三个圆柱体。圆柱体的大小可以通过传入scale参数控制
	 * 默认scale=1.0
	 * 该函数也有重载方法，通过传入坐标（x,y,z）可以设置坐标轴的位置
	 */
	viewer->addCoordinateSystem(1.0);
 
	//初始化相机参数，使用户从默认的角度观察点云
	viewer->initCameraParameters();
	return (viewer);
}
/**
 * 通常，使用最多的点云类型是PointXYZRGB类型，而不是PointXYZ，它还包含了颜色信息
 * 撇开这个不讲，你也希望通过颜色标注一些特别的点云，使它在视窗中具有辨认性
 * PCLVisualizer提供这样的性能，为包含颜色信息的点云提供显示，或者为点云分配色彩
 * 在很多设备中，比如Microsoft Kinect ，都可以产生RGB点云数据
 * PCLVisualizer可以为每个点云着色并显示
 */
//显示rgb点云
pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	// 如果点云中没有RGB字段，PCLVisualizer就不知道使用什么颜色
	// 点类型不一定需要PointRGB类型，只要它有三个颜色字段即可
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
 
}
 
//
pcl::visualization::PCLVisualizer::Ptr customColorVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}
pcl::visualization::PCLVisualizer::Ptr normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals
)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	//可以传入一个颜色控制器
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}
pcl::visualization::PCLVisualizer::Ptr shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
 
	//---------------------------------------
	//-----Add shapes at other locations-----
	//---------------------------------------
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	/**  Add a plane from a set of given model coefficients
	  * \param[in] coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
	  * \param[in] id the plane id/name (default: "plane")
	  * \param[in] viewport (optional) the id of the new viewport (default: 0)
	  *
	  * \code
	  * // The following are given (or computed using sample consensus techniques)
	  * // See SampleConsensusModelPlane for more information
	  * // Eigen::Vector4f plane_parameters;
	  *
	  * pcl::ModelCoefficients plane_coeff;
	  * plane_coeff.values.resize (4);    // We need 4 values
	  * plane_coeff.values[0] = plane_parameters.x ();
	  * plane_coeff.values[1] = plane_parameters.y ();
	  * plane_coeff.values[2] = plane_parameters.z ();
	  * plane_coeff.values[3] = plane_parameters.w ();
	  *
	  * addPlane (plane_coeff);
	  bool
		addPlane (const pcl::ModelCoefficients &coefficients,
				  const std::string &id = "plane",
				  int viewport = 0);
	  */
	viewer->addPlane(coeffs, "plane");
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");
	return (viewer);
}
 
//************************************
// Method:    传入三个const指针，返回一个PCLVisualizer指针
// 通过不同的视口（ViewPort）绘制多个点云
// 利用不同的搜索半径，基于同一点云计算出对应不同半径的两组法线
// Returns:   pcl::visualization::PCLVisualizer::Ptr
// Parameter: pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud
// Parameter: pcl::PointCloud<pcl::Normal>::ConstPtr normals1
// Parameter: pcl::PointCloud<pcl::Normal>::ConstPtr normals2
//************************************
pcl::visualization::PCLVisualizer::Ptr viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
	pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	// --------------------------------------------------------
	// ----------------打开3D视窗，添加点云和法线--------------
	// --------------------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	//使用默认值初始化相机参数
	viewer->initCameraParameters();
 
 
	int v1(0);
	//创建视口
	/**创建一个视口从 [xmin,ymin] 到 [xmax,ymax].
	* 参数 xmin ：视口在X轴的最小值 (0.0 <= 1.0)
	* 参数 ymin ：视口在Y轴的最小值 (0.0 <= 1.0)
	* 参数 xmax ：视口在X轴的最大值 (0.0 <= 1.0)
	* 参数 ymax： 视口在Y轴的最大值 (0.0 <= 1.0)
	* 参数 viewport ： 新视口的id，传入的是个引用参数
	*
	* 注意：
	*如果当前窗口不存在渲染器，将创建一个，并且*视口将被设置为0 ('all')。如果一个或多个渲染
	*器存在，视口ID将被设置为渲染器的总数- 1
	*/
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
 
	//设置viewport的背景颜色，第四个参数为指定视口的id，如果不输入，默认为全部视口设置背景颜色
	//setBackgroundColor(const double &r, const double &g, const double &b, int viewport = 0);
	viewer->setBackgroundColor(0, 0, 0, v1);
 
	//为视口添加文本,添加文本有许多重载函数，可以设置颜色字体等属性
	/*
	addText (const std::string &text,
		int xpos, int ypos,
		const std::string &id = "", int viewport = 0);
		*/
	viewer->addText("Radius:0.01", 10, 10, "v1 text", v1);
 
	//创建一个颜色处理器
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	//为指定的viewport加入点云，颜色处理器，
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);
 
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius:0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);
 
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);
 
	//为点云添加法线，指定viewport显示
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);
 
	return (viewer);
}
 
unsigned int text_id = 0;
void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;
		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
		}
		text_id = 0;
	}
}
void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX() << ", "
			<< event.getY() << ")" << std::endl;
		char str[512];
		sprintf(str, "text#%03d", text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}
pcl::visualization::PCLVisualizer::Ptr interactionCustomizationVis()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	/**
	 *  /** 为键盘事件注册一个回调函数
		  * callback  传入一个函数指针，这个函数将成为键盘事件的回调函数
		  * cookie    传入的用户数据，用无类型指针表示
		  * return 返回一个连接对象，可以通过这个对象断开回调函数.
	inline boost::signals2::connection
		registerKeyboardCallback(void(*callback) (const pcl::visualization::KeyboardEvent&, void*), void* cookie = NULL)
	{
		return (registerKeyboardCallback(boost::bind(callback, _1, cookie)));
	}
	 */
	viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());
	return (viewer);
}
// ----------------
// -----主函数-----
// ----------------
int 
main(int argc, char** argv)
{
	// --------------------------------------
	// ----------命令行参数解析--------------
	// --------------------------------------
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	bool simple(false), rgb(false), custom_c(false), normals(false),
		shapes(false), viewports(false), interaction_customization(false);
	if (pcl::console::find_argument(argc, argv, "-s") >= 0)
	{
		simple = true;
		std::cout << "Simple visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
	{
		custom_c = true;
		std::cout << "Custom colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-r") >= 0)
	{
		rgb = true;
		std::cout << "RGB colour visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-n") >= 0)
	{
		normals = true;
		std::cout << "Normals visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-a") >= 0)
	{
		shapes = true;
		std::cout << "Shapes visualisation example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-v") >= 0)
	{
		viewports = true;
		std::cout << "Viewports example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-i") >= 0)
	{
		interaction_customization = true;
		std::cout << "Interaction Customization example\n";
	}
	else
	{
		printUsage(argv[0]);
		return 0;
	}
	// ------------------------------------
	// ----------创建示例点云数据----------
	// ------------------------------------
	pcl::PointCloud < pcl::PointXYZ > ::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Generating example point clouds.\n\n";
	/*
	 * 创建一个向z轴挤压的椭圆，颜色由按红绿蓝的顺序渐变
	 */
	std::uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5*std::cos(pcl::deg2rad(angle));
			basic_point.y = sinf(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);
 
			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 | static_cast<std::uint32_t>(g) << 8
				| static_cast<std::uint32_t>(b));
			//将rgb的数据保存在一个浮点数
			point.rgb = *reinterpret_cast<float*> (&rgb);
			point_cloud_ptr->points.push_back(point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}
	basic_cloud_ptr->width = basic_cloud_ptr->size();
	basic_cloud_ptr->height = 1;//无序点云
	point_cloud_ptr->width = point_cloud_ptr->size();
	point_cloud_ptr->height = 1;
 
	// ----------------------------------------------------------------
	// ----------------以搜索半径0.05计算表面法线----------------------
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);
	// ----------------------------------------------------------------
	// ----------------以搜索半径0.1计算表面法线----------------------
	// ----------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);
 
	pcl::visualization::PCLVisualizer::Ptr viewer;
	if (simple)
	{
		viewer = simpleVis(basic_cloud_ptr);
	}
	else if (rgb)
	{
		viewer = rgbVis(point_cloud_ptr);
	}
	else if (custom_c)
	{
		viewer = customColorVis(basic_cloud_ptr);
	}
	else if (normals)
	{
		viewer = normalsVis(point_cloud_ptr, cloud_normals2);
	}
	else if (shapes)
	{
		viewer = shapesVis(point_cloud_ptr);
	}
	else if (viewports)
	{
		viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
	}
	else if (interaction_customization)
	{
		viewer = interactionCustomizationVis();
	}
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
}
```