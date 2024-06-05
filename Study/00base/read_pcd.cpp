#include <pcl/io/pcd_io.h>   // PCL的PCD输入输出操作相关的头文件
#include <pcl/point_types.h> // PCL的点类型定义头文件
typedef pcl::PointXYZ PointT;               // 起别名，命名PointT为pcl::PointXYZ类型，代表包含XYZ坐标的点
typedef pcl::PointCloud<PointT> PointCloud; // 起别名，命名PointCloud为包含PointT类型点的点云数据结构
PointCloud::Ptr cloud(new PointCloud);      // 创建PointCloud类型的智能指针，指向新的点云对象
PointCloud::Ptr cloud2(new PointCloud);     // 创建PointCloud类型的智能指针，指向新的点云对象
PointCloud::Ptr cloud3(new PointCloud);     // 创建PointCloud类型的智能指针，指向新的点云对象
int main()
{
    // 读取PCD数据方法一
    pcl::PCDReader reader;
    reader.read("./data/rabbit.pcd", *cloud);
    std::cout << "cloud points: " << cloud->width * cloud->height << std::endl;
    // 读取PCD数据方法二
    pcl::io::loadPCDFile("./data/airplane.pcd", *cloud2);
    std::cout << "cloud2 points: " << cloud2->width * cloud2->height << std::endl;
    // 写入PCD文件方法一
    pcl::PCDWriter writer;
    writer.write("./data/test_pcd.pcd", *cloud, false);          // 以默认ASCII格式写入文件
    writer.writeBinary("./data/test_pcd.pcd", *cloud);           // 以二进制格式写入文件
    writer.writeBinaryCompressed("./data/test_pcd.pcd", *cloud); // 以压缩二进制格式写入文件
    // 写入PCD文件方法二
    pcl::io::savePCDFile("./data/test_pcd.pcd", *cloud, false);          // 保存点云数据到"rabbit_copy.pcd"文件
    pcl::io::savePCDFileASCII("./data/test_pcd.pcd", *cloud);            // 以ASCII格式保存点云数据到"rabbit_copy.pcd"文件
    pcl::io::savePCDFileBinary("./data/test_pcd.pcd", *cloud);           // 以二进制格式保存点云数据到"rabbit_copy.pcd"文件
    pcl::io::savePCDFileBinaryCompressed("./data/test_pcd.pcd", *cloud); // 以压缩的二进制格式保存点云数据到"rabbit_copy.pcd"文件
    // 点云合成方法一
    *cloud3 = *cloud + *cloud2;
    writer.write("./data/concat_cloud.pcd", *cloud3, false);
    // 点云合成方法二
    pcl::PointCloud<pcl::PointXYZ> cloud_c;
    cloud_c = *cloud + *cloud2;
    writer.write("./data/concat_cloud.pcd", *cloud3, false);
    return (0);
}