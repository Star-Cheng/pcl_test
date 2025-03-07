#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
typedef pcl::PointXYZ PointT;               // 起别名，命名PointT为pcl::PointXYZ类型，代表包含XYZ坐标的点
typedef pcl::PointCloud<PointT> PointCloud; // 起别名，命名PointCloud为包含PointT类型点的点云数据结构
PointCloud::Ptr cloud(new PointCloud);      // 创建PointCloud类型的智能指针，指向新的点云对象
int main()
{

    // 假设你已经有了一个PointCloud类型的点云变量cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/airplane.pcd", *cloud);

    // 定义相机的参数
    float angularResolution = (float)(1.0f * (M_PI / 180.0f)); // 角分辨率，以弧度为单位
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));   // 水平最大角度
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 垂直最大角度

    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
                                                                      cloud->sensor_origin_[1],
                                                                      cloud->sensor_origin_[2])) *
                                 Eigen::Affine3f(cloud->sensor_orientation_);
    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    // 坐标系类型，通常是相机坐标系。
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

    // 使用了Boost库中的shared_ptr智能指针来管理pcl::RangeImage对象的内存
    // boost::shared_ptr是一个智能指针，用来自动管理对象的生命周期。当指向的对象不再使用时，它会自动释放内存。这避免了手动管理内存的问题。
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);

    // 创建了一个pcl::RangeImage类型的引用rangeImage，指向range_image_ptr所管理的对象。通过这种方式，你可以使用rangeImage来访问和操作range_image_ptr指向的对象，就像使用普通对象一样。
    pcl::RangeImage &rangeImage = *range_image_ptr;

    // 从点云创建范围图像
    rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << rangeImage << "\n";

    /*
    cloud：源点云数据。
    angularResolution：图像的角分辨率。
    maxAngleWidth：图像的最大水平视角。
    maxAngleHeight：图像的最大垂直视角。
    sensorPose：定义了传感器（或相机）在世界坐标系中的位置和方向。
    coordinate_frame：坐标系类型，通常是相机坐标系。
    noiseLevel：噪声水平，用于图像处理。
    minRange：传感器可以检测的最小距离。
    borderSize：图像边框的大小。
    */
    return (0);
}