#!/bin/bash

# change to catkin workspace root
CATKIN_WS=$HOME/catkin_ws
# change to desired location
SDK_DIR=$HOME/Downloads

sudo apt-get install -y ros-indigo-ardrone-autonomy ros-indigo-ar-track-alvar ros-indigo-oculus-rviz-plugins ros-indigo-oculus-sdk

cd $CATKIN_WS/src
git clone https://github.com/tum-vision/tum_ardrone.git -b indigo-devel
cd $CATKIN_WS
rosdep install tum_ardrone
# catkin_make

# Download/Unpack Oculus SDK (currently using latest stable release: 0.2.5) 
cd $SDK_DIR
wget "https://static.oculus.com/sdk-downloads/ovr_sdk_linux_0.2.5c.tar.gz"
tar -xvzf ovr_sdk_linux_0.2.5c.tar.gz

cd OculusSDK
./ConfigurePermissionsAndPackages.sh
make

cp -a LibOVR/ $CATKIN_WS/src/oculus/oculus_driver/
cd $CATKIN_WS/src/oculus/oculus_driver/LibOVR
make # might return errors but they can likely be ignored
cd $CATKIN_WS
catkin_make

cd $CATKIN_WS/src
git clone https://github.com/dougvk/tum_simulator.git
cd ..
catkin_make

# Myo
sudo apt-get install -y ros-indigo-rosserial-server
cd $CATKIN_WS/src
git clone https://github.com/roboTJ101/ros_myo.git
cd ..
catkin_make