#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_plotter.h>
using namespace pcl;

int main(int argc, char **argv)
{
    std::cout << "Starting program...\n";
    std::cout << "PCL version: " << PCL_VERSION << std::endl;
    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/rabbit.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file rabbit.pcd\n");
        system("pause");
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud->size() << " points.\n";

    // 估计法线
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.2);    // 使用半径在查询点周围0.2范围内的所有邻元素
    ne.compute(*cloud_normals); // 计算法线
    std::cout << "Normals estimated.\n";

    // FPFH
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud_normals);
    pcl::search::KdTree<PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfh.setSearchMethod(tree1);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh.setRadiusSearch(0.05); // 半径必须大于估计法线时使用的半径
    fpfh.compute(*fpfhs);
    std::cout << "FPFH features computed.\n";

    // 显示某点的fhfh特征
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh", 60);
    std::cout << "FPFH histogram prepared.\n";

    // 可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 1, 0.4, "normals");
    viewer.addPointCloud(cloud, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
    std::cout << "Visualization prepared. Starting visualization loop...\n";

    while (!viewer.wasStopped())
    {
        plotter.plot();
        viewer.spin();
    }

    std::cout << "Program finished.\n";
    return 0;
}
