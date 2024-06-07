#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>   // PCL的PCD输入输出操作相关的头文件
#include <pcl/point_types.h> // PCL的点类型定义头文件
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef pcl::PCDReader PCDReader;
typedef pcl::PCDWriter PCDWriter;
pcl::PCDReader reader;
pcl::PCDWriter writer;
