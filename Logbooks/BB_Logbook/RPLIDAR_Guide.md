Here I put the main things to do to run the RPLIDAR

First download ROQ Jazzy if not done already. 
Install slam-toolbox and nav2 

``` 
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2
```

Then install RPLIDAR drrivers and verify which USB port is in use:

```
ls /dev/ttyUSB*
sudo apt install ros-jazzy-rplidar-ros

// THe run test
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0
```
