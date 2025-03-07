#include "../include/my_point_type.h"

int main()
{
    std::cout << "PCL version: " << PCL_VERSION << std::endl;
    PointCloud::Ptr cloud(new PointCloud);
    reader.read("./data/rabbit.pcd", *cloud);
    std::cout << "cloud points: " << cloud->width * cloud->height << std::endl;
    PCLVisualizer viewer;
    viewer.addPointCloud(cloud);
    viewer.spin();
    return 0;
}