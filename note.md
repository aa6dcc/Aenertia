python3 mqtt_serial_bridge.py

ros2 run serial_bridge serial_bridge --ros-args -p serial_port:=/dev/esp32

ros2 launch rplidar_ros rplidar_a1_launch.py   serial_port:=/dev/rplidar   serial_baudrate:=115200   frame_id:=laser_frame

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
