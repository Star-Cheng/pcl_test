# pcl_test
+ https://robot.czxy.com/docs/pcl/chapter02/range_image/
## install pcl

```shell
bash install.sh
```

## setup

```shell
bash setup.sh
```

## pcl_test.cpp

```C++
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int user_data;
void viewerOneOff(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    // FIXME: possible race condition here:
    user_data++;
}

int main()
{
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("bun4.pcd", *cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // blocks until the cloud is actually rendered
    viewer.showCloud(cloud);

    // use the following functions to get access to the underlying more advanced/powerful
    // PCLVisualizer

    // This will only get called once
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped())
    {
        // you can also do cool processing here
        // FIXME: Note that this is running in a separate thread from viewerPsycho
        // and you should guard against race conditions yourself...
        user_data++;
    }
    return 0;
}
```

## CMakeLists.txt

```shell
cmake_minimum_required(VERSION 2.6)
project(pcl_test)
set(CMAKE_BUILD_TYPE Debug)  # 设置编译类型为Debug
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries (pcl_test ${PCL_LIBRARIES})
install(TARGETS pcl_test RUNTIME DESTINATION bin)
```

## pcl install

+ Ubuntu配置PCL: <https://blog.csdn.net/weixin_41836738/article/details/121451965>

## Cmake Debug

```shell
CMAKE_BUILD_TYPE:STRING=Debug
```

## Openni2

```shell
sudo apt-get update
sudo apt-get install libopenni-dev libopenni2-dev
```
