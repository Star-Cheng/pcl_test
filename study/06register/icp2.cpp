#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
main(int argc, char **argv) {
    // 定义输入和输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // 随机填充无序点云
    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
              << std::endl;
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        std::cout << "    " <<
                  cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
                  cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;

    // 在点云上执行简单的刚性变换，将cloud_out中的x平移0.7f米，然后再次输出数据值。
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    // 打印这些点
    std::cout << "Transformed " << cloud_in->points.size() << " data points:"
              << std::endl;
    for (size_t i = 0; i < cloud_out->points.size(); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " <<
                  cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    // 创建IterativeClosestPoint的实例
    // setInputSource将cloud_in作为输入点云
    // setInputTarget将平移后的cloud_out作为目标点云
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
    // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
    // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
    // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
    std::cout << matrix << std::endl;
    return (0);
}