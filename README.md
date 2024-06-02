# pcl_test

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

```bash
cmake_minimum_required(VERSION 2.6)
project(pcl_test)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test pcl_test.cpp)

target_link_libraries (pcl_test ${PCL_LIBRARIES})

install(TARGETS pcl_test RUNTIME DESTINATION bin)
```

## pcl install

+ Ubuntu配置PCL: <https://blog.csdn.net/weixin_41836738/article/details/121451965>

```bash
sudo apt-get update  
sudo apt-get install git build-essential linux-libc-dev
sudo apt-get install cmake cmake-gui
sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common 
sudo apt-get install libflann1.9 libflann-dev
sudo apt-get install libeigen3-dev 这个需要自己下载正确版本安装，我的是3.3.7
sudo apt-get install libboost-all-dev
sudo apt-get install libvtk7.1p-qt
sudo apt-get install libvtk7.1p 
sudo apt-get install libvtk7-qt-dev（按照错误提示一步一步安装所需要的东西）
sudo apt-get install libqhull* libgtest-dev
sudo apt-get install freeglut3-dev pkg-config
sudo apt-get install libxmu-dev libxi-dev
sudo apt-get install mono-complete
sudo apt-get install openjdk-8-jdk openjdk-8-jre

git clone https://github.com/PointCloudLibrary/pcl.git 

cd pcl 
mkdir release 
cd release
cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \ -DBUILD_GPU=ON-DBUILD_apps=ON -DBUILD_examples=ON \ -DCMAKE_INSTALL_PREFIX=/usr .. 
make  
sudo make install

pcl_v
```

## Cmake Debug

```Cmake Cache
CMAKE_BUILD_TYPE:STRING=Debug
```

## Openni2

```bash
sudo apt-get update
sudo apt-get install libpcl-dev libopenni-dev libopenni2-dev
```
