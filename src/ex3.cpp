#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  

int main(int argc, char** argv) {  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  
    
    // 加载点云数据  
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/rabbit.pcd", *cloud) == -1) {  
        PCL_ERROR("Couldn't read file sample.pcd \n");  
        return (-1);  
    }  

    pcl::visualization::PCLVisualizer viewer("3D Viewer");  
    viewer.setBackgroundColor(0.1, 0.1, 0.1);  

    // 添加点云  
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");  

    // 添加坐标系统  
    viewer.addCoordinateSystem(1.0);  
    
    // 添加坐标轴标签  
    viewer.addText3D("X", pcl::PointXYZ(1.1, 0, 0), 0.1, 1.0, 0.0, 0.0, "X_label"); // 红色 X 标签  
    viewer.addText3D("Y", pcl::PointXYZ(0, 1.1, 0), 0.1, 0.0, 1.0, 0.0, "Y_label"); // 绿色 Y 标签  
    viewer.addText3D("Z", pcl::PointXYZ(0, 0, 1.1), 0.1, 0.0, 0.0, 1.0, "Z_label"); // 蓝色 Z 标签  

    // 主循环  
    while (!viewer.wasStopped()) {  
        viewer.spinOnce(100); // 更新可视化和处理用户输入  
    }  

    return 0;  
}  