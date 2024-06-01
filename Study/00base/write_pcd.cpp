#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false; // 设置为非稠密点云, 即点云数据中可能包含无效或缺失的点
    cloud.resize(cloud.width * cloud.height);
    for (auto &point : cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    pcl::io::savePCDFileASCII("data/test_pcd.pcd", cloud); // 保存点云数据到文件
    std::cout << "Saved " << cloud.size() << " data points to data/test_pcd.pcd." << std::endl;
    for (const auto &point : cloud)
        std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
    return (0);
}