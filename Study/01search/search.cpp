#include <pcl/io/pcd_io.h>   // PCL的PCD输入输出操作相关的头文件
#include <pcl/point_types.h> // PCL的点类型定义头文件
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
typedef pcl::PointXYZ PointT;               // 起别名, 命名PointT为pcl::PointXYZ类型, 代表包含XYZ坐标的点
typedef pcl::PointCloud<PointT> PointCloud; // 起别名, 命名PointCloud为包含PointT类型点的点云数据结构
PointCloud::Ptr cloud(new PointCloud);      // 创建PointCloud类型的智能指针, 指向新的点云对象   // 创建PointCloud类型的智能指针, 指向新的点云对象
int main()
{
    // 读取PCD数据方法一
    pcl::PCDReader reader;
    reader.read("./data/rabbit.pcd", *cloud);
    std::cout << "cloud points: " << cloud->width * cloud->height << std::endl;
    // KDTree
    // 创建一个KdTree对象用于空间搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // 需要有一个已经填充了数据的点云对象 cloud
    kdtree.setInputCloud(cloud);
    // 设置一个需要搜索的点 searchPoint
    pcl::PointXYZ searchPoint;
    searchPoint.x = 0.0f;
    searchPoint.y = 0.0f;
    searchPoint.z = 0.0f;
    // K近邻搜索
    int K_KD = 3;
    std::vector<int> pointIdxNKNSearch(K_KD);
    std::vector<float> pointNKNSquaredDistance(K_KD);
    // 执行搜索, 返回找到的点的数量
    if (kdtree.nearestKSearch(searchPoint, K_KD, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x
                      << " " << (*cloud)[pointIdxNKNSearch[i]].y
                      << " " << (*cloud)[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }
    // 半径搜索
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 128.0f; // 这里应该为 radius 赋值, 表示搜索半径
    // 执行搜索, 返回找到的点的数量
    int KdTreeRadiusSearch_Nums = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (KdTreeRadiusSearch_Nums > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].y
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
    return 0;
}