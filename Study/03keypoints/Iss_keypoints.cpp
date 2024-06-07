#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include "include/my_point_type.h"
// typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

// 点云可视化函数
// 显示原始点云（model）和提取的关键点（scene_keypoints）
void visualize_pcd(PointCloud::Ptr model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints)
{
    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> model_color(model, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_keypoint_color(scene_keypoints, 0, 0, 255);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(model, model_color, "model");
    viewer.addPointCloud(scene_keypoints, scene_keypoint_color, "scene_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "scene_keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

// 你的文件地址
const std::string filename = "./data/rabbit.pcd";

int main(int, char **argv)
{
    PointCloud::Ptr cloud(new PointCloud);

    // 读取点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *cloud) == -1)
    {
        pcl::console::print_error("Couldn't read file %s!\n", argv[1]);
        return (-1);
    }
    std::cout << "points: " << cloud->points.size() << std::endl;

    // ISS关键点检测器
    pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGB> iss_detector;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());

    // 设置ISS检测器的参数
    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(0.003f); // 显著半径
    iss_detector.setNonMaxRadius(0.002f);  // 非最大抑制半径
    iss_detector.setThreshold21(0.65);     // 用于设置二三特征值比的阈值
    iss_detector.setThreshold32(0.1);      // 用于设置三一特征值比的阈值
    iss_detector.setMinNeighbors(4);       // 邻域最小点数
    iss_detector.setNumberOfThreads(4);    // 并行处理的线程数
    iss_detector.setInputCloud(cloud);     // 设置输入点云
    iss_detector.compute(*keypoints);      // 计算关键点

    std::cout << "Number of ISS_3D points in the result: " << (*keypoints).points.size() << std::endl;
    pcl::io::savePCDFile("./data/keypoints_iss_3d.pcd", *keypoints, false);

    // 可视化
    visualize_pcd(cloud, keypoints);
    return 0;
}