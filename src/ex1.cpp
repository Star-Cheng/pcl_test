#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
// 包含相关头文件
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;               // 起别名，命名PointT为pcl::PointXYZ类型，代表包含XYZ坐标的点
typedef pcl::PointCloud<PointT> PointCloud; // 起别名，命名PointCloud为包含PointT类型点的点云数据结构
PointCloud::Ptr cloud(new PointCloud);      // 创建PointCloud类型的智能指针，指向新的点云对象
int main()
{
    // 读取点云
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("./data/rabbit.pcd", *cloud1);

    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("./data/airplane.pcd", *cloud2);

    // 定义对象
    pcl::visualization::PCLVisualizer viewer;
    // 设置背景颜色，默认黑色
    viewer.setBackgroundColor(0, 0, 0); // rgb

    // --- 显示点云数据 ----
    // "cloud1" 为显示id，默认cloud,显示多个点云时用默认会报警告。
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud2, 0, 255, 0); // rgb
    viewer.addPointCloud(cloud1, green, "cloud1");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud2, 255, 0, 0); // rgb
    // 将点云设置颜色，默认白色
    viewer.addPointCloud(cloud2, red, "cloud2");

    // 将两个点连线
    PointT temp1 = cloud1->points[0];
    PointT temp2 = cloud1->points[1000];
    // viewer.addLine(temp1, temp2, "line0");
    //  同样可以设置线的颜色，蓝色
    viewer.addLine(temp1, temp2, 0, 0, 255, "line0");
    viewer.addCoordinateSystem(1.0); 
    // --- 显示网格数据 ---
    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile("./data/bun000.ply", mesh);
    pcl::PLYReader readerply;
    readerply.read("./data/bun000.ply", *cloud);
    viewer.addPolygonMesh(mesh);

    // 开始显示2种方法,任选其一
    // 1. 阻塞式
    viewer.spin();

    // 2. 非阻塞式
    while (!viewer.wasStopped())
    {
        // viewer.spinOnce(100);
        // 可添加其他操作
    }
    system("pause");
    return 0;
}
