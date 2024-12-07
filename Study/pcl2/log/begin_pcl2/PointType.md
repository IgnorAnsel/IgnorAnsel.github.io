@[3.PCL点云数据类型](这里写自定义目录标题)


## 1. PointXYZ

```CPP
template<typename PointT>
struct PointXYZ
{
  float x;
  float y;
  float z;
};
```
## 2. PointXYZI
```CPP
template<typename PointT>
struct PointXYZI
{
  float x;
  float y;
  float z;
  uint32_t intensity;
};
```
## 3. PointXYZRGB

```CPP
template<typename PointT>
struct PointXYZRGB
{
  float x;
  float y;
  float z;
  uint8_t r; // Red
  uint8_t g; // Green
  uint8_t b; // Blue
};
```
## 4. PointXYZRGBA

```CPP
template<typename PointT>
struct PointXYZRGBA
{
  float x;
  float y;
  float z;
  uint8_t r; // Red
  uint8_t g; // Green
  uint8_t b; // Blue
  uint8_t a; // Alpha
};
```
## 5. PointXYZRGBNormal

```CPP
template<typename PointT>
struct PointXYZRGBNormal
{
  float x;
  float y;
  float z;
  uint8_t r; // Red
  uint8_t g; // Green
  uint8_t b; // Blue
  float normal_x; // Normal 法线
  float normal_y;
  float normal_z;
  float curvature; // 曲率 
};
```
## 6. PointXYZINormal

```CPP
template<typename PointT>
struct PointXYZINormal
{
  float x;
  float y;
  float z;
  uint32_t intensity;
  float normal_x;
  float normal_y;
  float normal_z;
  float curvature;
};
```
## 7. PointXY
```CPP
template<typename PointT>
struct PointXY
{
  float x;
  float y;
};
```

## 8. Normal

```CPP
template<typename PointT>
struct Normal
{
  float normal_x;
  float normal_y;
  float normal_z;
  float curvature;
};
```


## 9. PointWithRange

```CPP
template<typename PointT>
struct PointWithRange
{
  float x;
  float y;
  float z;
  float range;
};
```

## 10. PointWithViewpoint

```CPP
template<typename PointT>
struct PointWithViewpoint
{
  float x;
  float y;
  float z;
  float viewpoint_x; // vp_x;
  float viewpoint_y; // vp_y;
  float viewpoint_z; // vp_z;
};
```
## 更多
[更多介绍](https://blog.csdn.net/u013925378/article/details/83537844)
