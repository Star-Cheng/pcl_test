#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/table_scene_lms400.pcd", *cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(pcl::search::Search<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    ne.setKSearch(20);
    ne.compute(*normals);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    // viewer.addPointCloud(cloud, "cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
    
    while (!viewer.wasStopped())
    {
        // viewer.spinOnce();
        viewer.spin();
    }
    
    return 0;
}