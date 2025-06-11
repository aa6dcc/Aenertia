DISCLAIMER!!!

I had a lot of trouble getting this to work. Be very careful of getting the WSL to work correctly.
To do so creat a file ".wslconfig" in c:/users/(username)/. using notpad.

Then in that file add two lines: 

```
[wsl2]
networkingMode=mirrored
```

This allows connectivity between the pi and wsl. I tried every possible thing to make it work otherwise this is the only single fix. 

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


## Setup

First remeber to edit the ~/.bashrc using nano. Edit the lines at the end so that every terminal sources ROS2 correctly

You want to source the following:

Change the line at the end by doing 
```
# On the PI
source /opt/ros/jazzy/setup.bash
source ~/ws_lidar/install/setup.bash

# On the laptop for Rviz
source /opt/ros/jazzy/setup.bas
```

Then run ``source ~/.bashrc`` to apply changes


It is useful to verify 

```
echo $RMW_IMPLEMENTATION    # should print: rmw_cyclonedds_cpp
echo $ROS_DOMAIN_ID         # should be empty (or 42 if you set it)
```

## Running the Slamtool box and rplidar

If you followed the explaination I made in ROS_Installs.md You should be able to run the following:
```
# On the Pi, start LIDAR:
ros2 launch rplidar_ros view_rplidar_a1_launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    frame_id:=laser &

# On the Pi, start SLAM:
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    namespace:=slam \
    slam_params_file:="$(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/config/mapper_params_online.yaml"

# On your laptop, start rviz2
rviz2


```
