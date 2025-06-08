#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from server.database.dynamodb import update_robot_state, log_location, find_nearest_key_location
from datetime import datetime
import math

last_pose = None
last_time = None

def get_velocity(current_pose, current_time):
    global last_pose, last_time
    if last_pose is None or last_time is None:
        last_pose = current_pose
        last_time = current_time
        return 0.0

    dx = current_pose.position.x - last_pose.position.x
    dy = current_pose.position.y - last_pose.position.y
    dt = (current_time - last_time).to_sec()
    velocity = math.sqrt(dx**2 + dy**2) / dt if dt > 0 else 0.0

    last_pose = current_pose
    last_time = current_time
    return velocity

def pose_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    now = rospy.Time.now()
    velocity = get_velocity(msg.pose, now)

    from server.database.dynamodb import get_battery_level, get_mode, get_status

    device_id = "raspi-001"
    battery = get_battery_level()
    mode = get_mode()
    status = get_status(velocity)
    current_key_loc = find_nearest_key_location(x, y)

    update_robot_state(device_id, x, y, battery, mode, status, current_key_loc)
    log_location(device_id, x, y)
    rospy.loginfo(f"Sent ({x:.2f}, {y:.2f}), battery {battery:.1f}%, status: {status}")

def main():
    rospy.init_node("slam_dynamodb_bridge", anonymous=True)
    rospy.Subscriber("/slam_toolbox/pose", PoseStamped, pose_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
