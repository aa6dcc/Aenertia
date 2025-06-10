## Setup

First remeber to edit the ~/.bashrc

You want to source the following:

Change the line at the end by doing 
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Needed because of connectivity issues I had

# export ROS_DOMAIN_ID=42
# Only do the above if needed (When we have many nodes)

# Source the ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source the ws_lidar workspace (where rplidar_ros & slam_toolbox live)
source ~/ws_lidar/install/setup.bash
```

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
