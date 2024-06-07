#include "include/my_point_type.h"

int main()
{
    PointCloud::Ptr cloud(new PointCloud);
    reader.read("./data/rabbit.pcd", *cloud);
    std::cout << "cloud points: " << cloud->width * cloud->height << std::endl;
    PCLVisualizer viewer;
    viewer.addPointCloud(cloud);
    viewer.spin();
    return 0;
}