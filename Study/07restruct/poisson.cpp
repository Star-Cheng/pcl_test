#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <string.h>
#include <string>

int main(int argc, char** argv)
{
    std::string input_file = "/root/code/pcl/pcl_test/data/rabbit.pcd";

    // Load the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        PCL_ERROR("Could not read pcd file!\n");
        return -1;
    }

    // Calculate normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate point cloud and normals
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Create KD tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Create Poisson object and set parameters
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false);
    pn.setDegree(2);
    pn.setDepth(8);
    pn.setIsoDivide(8);
    pn.setManifold(false);
    pn.setOutputPolygons(false);
    pn.setSamplesPerNode(3.0);
    pn.setScale(1.25);
    pn.setSolverDivide(8);

    // Set search method and input point cloud
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);

    // Create polygon mesh to store result
    pcl::PolygonMesh mesh;
    // Perform reconstruction
    pn.performReconstruction(mesh);

    // Display result
    pcl::visualization::PCLVisualizer viewer("Poisson Surface Reconstruction");
    viewer.addPolylineFromPolygonMesh(mesh);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}