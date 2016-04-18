# oculus_drone
Package for operating Parrot AR.Drone 2.0 with Oculus Rift and Myo Gesture Control Armband
## Run
Both launch files create all required nodes for runtime
* Oculus View Manager
* Oculus Driver
* Joystick Node
    * Recognizes if Logitech or PS3 joystick is connected and maps buttons accordingly
* Controller Button Mapping Node
* ardrone_autonomy node OR tum_simulator node

### Gazebo Simulation
```
roslaunch drone_vr simulate.launch
```
### Phyiscal AR.Drone
```
roslaunch drone_vr ardrone.launch
```
