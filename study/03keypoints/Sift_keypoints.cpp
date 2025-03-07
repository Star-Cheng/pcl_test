#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
using namespace std;

namespace pcl
{
    template <>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator()(const PointXYZ &p) const
        {
            return p.z;
        }
    };
}

int main(int argc, char *argv[])
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/rabbit.pcd", *cloud_xyz) == -1)
    {
        PCL_ERROR("Couldn't read file1 \n");
        return (-1);
    }
    /*关键点检测的阈值设置不合适：如果min_contrast设置得太高，可能会导致没有关键点被检测到。
    试着将其降低到更小的值。尺度参数设置不合适：min_scale、n_octaves和n_scales_per_octave
    参数定义了SIFT算法中的尺度空间，如果设置不当，同样会影响关键点的检测。您可能需要针对您的数据调整这些参数。*/
    const float min_scale = 0.0001;    // 设置尺度空间中最小尺度的标准偏差
    const int n_octaves = 6;           // 设置高斯金字塔组（octave）的数目
    const int n_scales_per_octave = 4; // 设置每组（octave）计算的尺度
    const float min_contrast = 0.0001; // 设置限制关键点检测的阈值

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift; // 创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;
    sift.setInputCloud(cloud_xyz); // 设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);                                // 创建一个空的kd树对象tree，并把它传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave); // 指定搜索关键点的尺度范围
    sift.setMinimumContrast(min_contrast);                     // 设置限制关键点检测的阈值
    sift.compute(result);                                      // 执行sift关键点检测，保存结果在result

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp); // 将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据

    // 可视化输入点云和关键点
    pcl::visualization::PCLVisualizer viewer("Sift keypoint");
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addPointCloud(cloud_xyz, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
    viewer.addPointCloud(cloud_temp, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}