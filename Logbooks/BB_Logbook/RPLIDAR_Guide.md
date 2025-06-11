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

## Initial ROS setup

FIgure out how to dowload ROS Jazzy Slam_toolbox and Nav2 yourself it is well documented

Now you need to create a working directory such as "ws_lidar". We clone the RPLIDAR packages  for ROS2 in it

```
cd ~
mkdir -p ws_lidar/src
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git rplidar_ros
cd ~/ws_lidar

bb923@nihil:~/ws_lidar/src$ cd ~/ws_lidar
bb923@nihil:~/ws_lidar$ colcon build --symlink-install
```

