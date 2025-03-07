// https://pcl.readthedocs.io/projects/tutorials/en/master/octree_change.html#octree-change-detection
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>

int main()
{
    srand((unsigned int)time(NULL));

    // Octree resolution - side length of octree voxels
    float resolution = 32.0f;

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data for cloudA
    cloudA->width = 10;
    cloudA->height = 1;
    cloudA->points.resize(cloudA->width * cloudA->height);

    for (std::size_t i = 0; i < cloudA->size(); ++i)
    {
        (*cloudA)[i].x = 64.0f;
        (*cloudA)[i].y = 64.0f;
        (*cloudA)[i].z = 64.0f;
    }

    // 添加点云到八叉树中，构建八叉树
    octree.setInputCloud(cloudA);     // 设置输入点云
    octree.addPointsFromInputCloud(); // 将点云中的点添加到八叉树数据结构中

    // 在切换缓冲区时重置八叉树但保持先前的树结构在内存中
    octree.switchBuffers();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

    // 为cloudB创建点云
    cloudB->width = 11;
    cloudB->height = 1;
    cloudB->points.resize(cloudB->width * cloudB->height);

    for (std::size_t i = 0; i < cloudB->size() - 1; ++i)
    {
        (*cloudB)[i].x = 64.0f;
        (*cloudB)[i].y = 64.0f;
        (*cloudB)[i].z = 64.0f;
    }
    (*cloudB)[10].x = 63.0f;
    (*cloudB)[10].y = 63.0f;
    (*cloudB)[10].z = 63.0f;
    // 添加cloudB到八叉树中
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();
    /********************************************************************************************
    为了检索获取存在于couodB的点集R，此R并没有cloudA中的元素，可以调用getPointIndicesFromNewVoxels方法，
    通过探测两个八叉树之间体素的不同，它返回cloudB 中新加点的索引的向量, 通过索引向量可以获取R点集很明显这样
    就探测了cloudB相对于cloudA变化的点集，但是只能探测到在cloudA上增加的点集，二不能探测减少的
    *********************************************************************************************/
    std::vector<int> newPointIdxVector;

    // Get vector of point indices from octree voxels which did not exist in previous buffer
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // Output points
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (std::size_t i = 0; i < newPointIdxVector.size(); ++i)
    {
        std::cout << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << (*cloudB)[newPointIdxVector[i]].x << " "
                  << (*cloudB)[newPointIdxVector[i]].y << " "
                  << (*cloudB)[newPointIdxVector[i]].z << std::endl;
    }
}