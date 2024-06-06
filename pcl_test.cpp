#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>             // VoxelGrid体素滤波器
#include <pcl/filters/passthrough.h>            // PassThrough滤波器
#include <pcl/filters/approximate_voxel_grid.h> // ApproximateVoxelGrid体素滤波器
#include <pcl/filters/uniform_sampling.h>       // UniformSampling均匀采样滤波器
#include <pcl/filters/statistical_outlier_removal.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr approximate_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("./data/airplane.pcd", *cloud);
    std::cout << "cloud points: " << cloud->width * cloud->height << std::endl;

    // 直通滤波
    /*直通滤波器的工作原理类似于一个空间的“切片”操作。你可以指定一个轴（例如X轴、Y轴或Z轴）和该轴上的
    一个最小值与最大值，滤波器将仅保留位于这个范围内的点，而移除所有其他的点。例如，如果你在Z轴上应用直
    通滤波器，并设置最小值为0.5米，最大值为1.5米，那么所有Z值不在0.5米到1.5米范围内的点都将被移除*/
    pcl::PassThrough<pcl::PointXYZ> pass; // 实例化一个直通滤波对象
    pass.setInputCloud(cloud);            // 设置输入点云
    pass.setFilterFieldName("z");         // 设置要进行直通滤波的字段为Z轴
    pass.setFilterLimits(0.0, 1.0);       // 设置Z轴的过滤范围为0.0到1.0

    // pass.setFilterLimitsNegative (true); // 如果启用，则会保留指定范围之外的点
    pass.filter(*pass_filtered); // 执行滤波操作，过滤后的点云存储在cloud_filtered

    // 体素滤波
    // VoxelGrid体素滤波器
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    // 设置输入点云为cloud
    sor.setInputCloud(cloud);
    // 设置每个体素的大小为0.01米（在X，Y，Z方向）。这个大小决定了滤波的粒度。
    sor.setLeafSize(0.05f, 0.05f, 0.1f);
    // 执行滤波操作，结果存储在cloud_filtered中
    sor.filter(*voxel_filtered);

    // Approximate 体素格滤波器
    // 创建一个ApproximateVoxelGrid对象approximate_voxel_filter，这个滤波器用于处理pcl::PointXYZ类型的点云
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // 设置每个体素的大小为0.2米（在X，Y，Z方向）
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    // 设置输入点云为input_cloud
    approximate_voxel_filter.setInputCloud(cloud);
    // 执行滤波操作，结果存储在fapproximate_voxel_filtered中
    approximate_voxel_filter.filter(*approximate_voxel_filtered);
    // 创建了一个UniformSampling对象，它是模板化的，这里用的模板参数是pcl::PointXYZ，意味着它将作用于包含XYZ坐标的点云
    pcl::UniformSampling<pcl::PointXYZ> filter;

    // 均值滤波
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(0.02); // 采样间隔为0.02
    uniform_sampling.filter(*uniform_filtered);

    // 可视化滤波后的点云
    // 定义对象
    pcl::visualization::PCLVisualizer viewer;
    int v1(1);                                      // viewport
    viewer.createViewPort(0.0, 0.5, 0.25, 1.0, v1); // 创建一个viewport, 左下角坐标和右上角坐标以及显示的viewport id
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.addPointCloud(cloud, "Original", v1);

    int v2(2); // viewport
    viewer.createViewPort(0.25, 0.5, 0.5, 1.0, v2);
    viewer.setBackgroundColor(0, 0, 0, v2);
    viewer.addPointCloud(pass_filtered, "PassThrough", v2);

    int v3(3); // viewport
    viewer.createViewPort(0.5, 0.5, 0.75, 1.0, v3);
    viewer.setBackgroundColor(0, 0, 0, v3);
    viewer.addPointCloud(voxel_filtered, "Voxel_filtered", v3);

    int v4(4); // viewport
    viewer.createViewPort(0.75, 0.5, 1.0, 1.0, v4);
    viewer.setBackgroundColor(0, 0, 0, v4);
    viewer.addPointCloud(uniform_filtered, "Approximate_voxel_filtered", v4);

    int v5(5);                                      // viewport
    viewer.createViewPort(0.0, 0.0, 0.25, 0.5, v5); // 创建一个viewport, 左下角坐标和右上角坐标以及显示的viewport id
    viewer.setBackgroundColor(0, 0, 0, v5);
    viewer.addPointCloud(cloud, "Original2", v5);

    int v6(6); // viewport
    viewer.createViewPort(0.25, 0.0, 0.5, 0.5, v6);
    viewer.setBackgroundColor(0, 0, 0, v6);
    viewer.addPointCloud(pass_filtered, "PassThrough2", v6);

    int v7(7); // viewport
    viewer.createViewPort(0.5, 0.0, 0.75, 0.5, v7);
    viewer.setBackgroundColor(0, 0, 0, v7);
    viewer.addPointCloud(voxel_filtered, "Voxel_filtered2", v7);

    int v8(8); // viewport
    viewer.createViewPort(0.75, 0.0, 1.0, 0.5, v8);
    viewer.setBackgroundColor(0, 0, 0, v8);
    viewer.addPointCloud(uniform_filtered, "Approximate_voxel_filtered2", v8);

    viewer.spin();

    return (0);
}