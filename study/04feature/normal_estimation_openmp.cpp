#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <boost/thread/thread.hpp>

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./data/airplane.pcd", *cloud);
    std::cout << "PCL version: " << PCL_VERSION << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;//创建法线估计向量
    ne.setNumberOfThreads(omp_get_max_threads());
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//创建一个空的KdTree对象，并把它传递给法线估计向量，基于给出的输入数据集，KdTree将被建立
    ne.setSearchMethod(tree);
    ne.setViewPoint(10, 10, 10);//设置视点，官网上的pcl::flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z, Eigen::Vector4f &normal)是对“一个已知点”的法线进行手动定向
    //ne.setViewPoint(-10, -10, -10);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);//存储输出数据
    ne.setRadiusSearch(0.05);        //半径单位为毫米
    // ne.setKSearch(5);
    ne.compute(*cloud_normals);

    //保存具有法线信息的点云文件
    // pcl::PCDWriter savePCDFile;
    // savePCDFile.write("./data/rabbit.pcd", *cloud_normals);

    //可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer(new pcl::visualization::PCLVisualizer());

    int v1(0);
    m_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    m_viewer->addCoordinateSystem(v1);
    m_viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
    m_viewer->addPointCloud(cloud, color, "cloud1", v1);

    int v2(0);
    m_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    m_viewer->addCoordinateSystem(v2);
    m_viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    //显示点云及其法线
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Cap_Cloud_Point(cloud, 255, 0, 0);
    m_viewer->addPointCloud(cloud, Cap_Cloud_Point, "cloud2", v2);
    //显示点云法线，第三个参数是法线显示的个数(每2个点显示1个)，第四个参数是法线的长度(此处为0.5)
    m_viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 5, 0.1, "normals", v2); 
    while (!m_viewer->wasStopped())
    {
        m_viewer->spin();
    }
    return 0;
}