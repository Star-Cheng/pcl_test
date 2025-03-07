// https://pcl.readthedocs.io/projects/tutorials/en/master/octree.html#octree-search
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <vector>
#include <ctime>

int main()
{
    srand((unsigned int)time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        (*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    /***************************************************************************************
    resolution表示八叉树的分辨率，即每个八叉树节点所覆盖的空间大小。在这里，分辨率为128.0f，表示八叉树将空间分割为大小为128x128x128的立方体节点。
    更大的分辨率会导致八叉树节点更小，相应地会增加八叉树的深度和节点数量，而更小的分辨率会导致八叉树节点更大，减少深度和节点数量
    ***************************************************************************************/
    float resolution = 128.0f; // resolution表示八叉树的分辨率，即每个八叉树节点所覆盖的空间大小

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud(); // 将点云中的点添加到八叉树数据结构中

    pcl::PointXYZ searchPoint;

    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // Neighbors within voxel search

    std::vector<int> pointIdxVec;
    /***************************************************************************************
    voxelSearch(searchPoint, pointIdxVec)：该函数用于在八叉树中进行体素搜索。给定一个搜索点searchPoint，
    函数会找到包含该搜索点的体素，并返回该体素中的所有点的索引，这些点保存在pointIdxVec中。
    ***************************************************************************************/
    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at ("
                  << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")"
                  << std::endl;

        for (std::size_t i = 0; i < pointIdxVec.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxVec[i]].x
                      << " " << (*cloud)[pointIdxVec[i]].y
                      << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
    }

    // K nearest neighbor search

    int K = 10;

    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;
    /***************************************************************************************
    nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)：该函数用于在八叉树
    中搜索K个最近邻的点。给定一个搜索点searchPoint和要搜索的最近邻点的数量K，函数会返回K个最近邻点的索引和对应
    的距离，分别保存在pointIdxNKNSearch和pointNKNSquaredDistance中。
    ***************************************************************************************/
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x
                      << " " << (*cloud)[pointIdxNKNSearch[i]].y
                      << " " << (*cloud)[pointIdxNKNSearch[i]].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // Neighbors within radius search

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;
    /***************************************************************************************
    radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)：该函数
    用于在八叉树中搜索在给定半径范围内的点。给定一个搜索点searchPoint和搜索半径radius，函数会返回所有距离该搜索
    点在给定半径范围内的点的索引和对应的距离，分别保存在pointIdxRadiusSearch和pointRadiusSquaredDistance中。
    ***************************************************************************************/
    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].y
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
}