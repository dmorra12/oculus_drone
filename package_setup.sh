#!/bin/bash
sudo apt-get install -y ros-indigo-ardrone-autonomy ros-indigo-ar-track-alvar ros-indigo-oculus-rviz-plugins ros-indigo-oculus-sdk

cd ~catkin_ws/src
git clone https://github.com/tum-vision/tum_ardrone.git -b indigo-devel
cd ..
rosdep install tum_ardrone
# catkin_make

# Download/Unpack Oculus SDK (currently using latest stable release: 0.2.5) 
cd ~/Downloads
wget "https://static.oculus.com/sdk-downloads/ovr_sdk_linux_0.2.5c.tar.gz"
tar -xvzf ovr_sdk_linux_0.2.5c.tar.gz

cd OculusSDK
./ConfigurePermissionsAndPackages.sh
make

cp -a LibOVR/ ~/catkin_ws/src/oculus/oculus_driver/
cd ~/catkin_ws/src/oculus/oculus_driver/LibOVR
make #(might return errors but they can be ignored)
cd ~/catkin_ws
catkin_make