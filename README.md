# oculus_drone
Package for operating Parrot AR.Drone 2.0 with Oculus Rift DK1 and Myo Gesture Control Armband

[Brief YouTube video demo](https://www.youtube.com/watch?v=posDNzym2Gc)

This ROS package was developed for a course at Johns Hopkins University titled "Robot Systems Programming." More information about the course can be found [here](https://dscl.lcsr.jhu.edu/ME530707_2016).

## Required Prerequisites
* [OTL/oculus](https://github.com/OTL/oculus)
* [Oculus SDK for Linux v0.2.5](https://developer.oculus.com/downloads/pc/0.2.5/Oculus_SDK_for_Linux/)
* [roboTJ101/ros_myo](https://github.com/roboTJ101/ros_myo)
* [tum-vision/tum_ardrone](https://github.com/tum-vision/tum_ardrone)
* [tum-vision/tum_simulator](https://github.com/tum-vision/tum_simulator)
* [joy](http://wiki.ros.org/joy)

## Run
Both launch files create all required nodes for runtime

NOTE: Node automatically recognizes if Logitech or PS3 gamepad controller is connected and maps buttons accordingly

### Phyiscal AR.Drone
Oculus/Myo controls
```
roslaunch drone_vr ardrone.launch
```
Gamepad only operation
```
roslaunch drone_vr ardrone_joy_only.launch
```
### Gazebo Simulation
Oculus/Myo controls
```
roslaunch drone_vr simulate.launch
```
Gamepad only operation
```
roslaunch drone_vr simulate_joy_only.launch
```
