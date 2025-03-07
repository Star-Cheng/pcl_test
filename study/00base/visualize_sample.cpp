#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZ PointT;               // 起别名，命名PointT为pcl::PointXYZ类型，代表包含XYZ坐标的点
typedef pcl::PointCloud<PointT> PointCloud; // 起别名，命名PointCloud为包含PointT类型点的点云数据结构
PointCloud::Ptr cloud(new PointCloud);      // 创建PointCloud类型的智能指针，指向新的点云对象
int main()
{
    pcl::PCDReader reader;
    reader.read("./data/rabbit.pcd", *cloud);
    std::cout << "cloud points: " << cloud->width * cloud->height << std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer"); // 创建viewer对象
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
    return (0);
}