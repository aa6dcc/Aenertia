ros2 run rplidar_ros rplidar_composition   --ros-args     -p serial_port:=/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0     -p serial_baudrate:=115200


ros2 run serial_bridge serial_bridge   --ros-args     -p serial_port:=/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0

ros2 run rplidar_ros rplidar_composition   --ros-args     -p serial_port:=/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0      -p serial_baudrate:=115200




ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0 odom base_link

ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0 odom base_link

ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0 odom base_link

ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0 odom base_link

ros2 run tf2_ros static_transform_publisher   0 0 0 0 0 0 odom base_link

ros2 launch balancing_bot_description bringup.launch.py

