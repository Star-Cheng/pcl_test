#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/io.h> // for concatenateFields
#include <pcl/io/pcd_io.h>
int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/rabbit.pcd", *cloud_a) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_a->width * cloud_a->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (const auto &point : *cloud_a)
    std::cout << " " << point.x
              << " " << point.y
              << " " << point.z << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/airplane.pcd", *cloud_b) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_b->width * cloud_b->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (const auto &point : *cloud_b)
    std::cout << " " << point.x
              << " " << point.y
              << " " << point.z << std::endl;

  pcl::PointCloud<pcl::PointXYZ> cloud_c;
  // Fill in the cloud data
  cloud_c.width = cloud_a->width + cloud_b->width;
  cloud_c.height = 1;
  cloud_c.is_dense = false; // 设置为非稠密点云, 即点云数据中可能包含无效或缺失的点
  cloud_c.resize(cloud_c.width * cloud_c.height);
 // 合并点云
  size_t i = 0;
  for (const auto &point : *cloud_a)
  {
    cloud_c.points[i++] = point;
  }
  for (const auto &point : *cloud_b)
  {
    cloud_c.points[i++] = point;
  }
  pcl::io::savePCDFileASCII("data/concat_cloud.pcd", cloud_c); // 保存点云数据到文件
  std::cout << "Saved " << cloud_c.size() << " data points to data/concat_cloud.pcd." << std::endl;
  for (const auto &point : cloud_c)
      std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
  return (0);
}