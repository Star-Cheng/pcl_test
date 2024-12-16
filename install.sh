# Ubuntu配置PCL: https://blog.csdn.net/weixin_41836738/article/details/121451965
#!/usr/bin/env bash
apt-get update
apt-get install gdb -y
apt-get install git build-essential linux-libc-dev -y
apt-get install cmake cmake-gui -y
apt-get install libusb-1.0-0-dev libusb-dev libudev-dev -y
apt-get install mpi-default-dev openmpi-bin openmpi-common  -y
apt-get install libflann1.9 libflann-dev -y
apt-get install libeigen3-dev==3.3.7 -y
apt-get install libboost-all-dev -y
apt-get install libvtk7.1p-qt -y
apt-get install libvtk7.1p  -y
apt-get install libvtk7-qt-dev -y
apt-get install libqhull* libgtest-dev -y
apt-get install freeglut3-dev pkg-config -y
apt-get install libxmu-dev libxi-dev -y
apt-get install mono-complete -y
apt-get install openjdk-8-jdk openjdk-8-jre -y
apt-get install libopenni-dev libopenni2-dev -y
mkdir -p ~/download/pcl && cd ~/download/pcl
git clone https://github.com/PointCloudLibrary/pcl.git && cd pcl 
mkdir -p release && cd release
cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \ -DBUILD_GPU=ON-DBUILD_apps=ON -DBUILD_examples=ON \ -DCMAKE_INSTALL_PREFIX=/usr .. 
make -j16
make install