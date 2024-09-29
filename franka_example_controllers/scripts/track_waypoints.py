#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import numpy as np
import tf.transformations

pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

# Example waypoints
waypoints = []

def create_waypoints():
    # Create a few example waypoints for demonstration purposes
    waypoint1 = PoseStamped()
    waypoint1.pose.position.x = 0.35
    waypoint1.pose.position.y = 0.1
    waypoint1.pose.position.z = 0.8
    waypoint1.pose.orientation.x = 1.0
    waypoint1.pose.orientation.y = 0.0
    waypoint1.pose.orientation.z = 0.0
    waypoint1.pose.orientation.w = 0.0

    waypoint2 = PoseStamped()
    waypoint2.pose.position.x = 0.4
    waypoint2.pose.position.y = -0.1
    waypoint2.pose.position.z = 0.8
    waypoint2.pose.orientation.x = 1.0
    waypoint2.pose.orientation.y = 0.0
    waypoint2.pose.orientation.z = 0.0
    waypoint2.pose.orientation.w = 0.0

    waypoint3 = PoseStamped()
    waypoint3.pose.position.x = 0.45
    waypoint3.pose.position.y = 0.0
    waypoint3.pose.position.z = 0.8
    waypoint3.pose.orientation.x = 1.0
    waypoint3.pose.orientation.y = 0.0
    waypoint3.pose.orientation.z = 0.0
    waypoint3.pose.orientation.w = 0.0

    # Add waypoints to list
    waypoints.extend([waypoint1, waypoint2, waypoint3])


def publish_waypoint(pose):
    # Ensures the published pose stays within the defined limits
    pose.pose.position.x = max([min([pose.pose.position.x, position_limits[0][1]]), position_limits[0][0]])
    pose.pose.position.y = max([min([pose.pose.position.y, position_limits[1][1]]), position_limits[1][0]])
    pose.pose.position.z = max([min([pose.pose.position.z, position_limits[2][1]]), position_limits[2][0]])

    print(f"Publishing waypoint: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}")
    pose_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("waypoint_publisher_node")
    link_name = rospy.get_param("~link_name", "panda_2_gripper_origin")

    # Create a publisher for PoseStamped
    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=10)

    # Initialize waypoints
    create_waypoints()

    # Loop through the waypoints and publish them one by one
    rate = rospy.Rate(0.5)  # Publish each waypoint every second
    waypoint_index = 0
    while not rospy.is_shutdown():
        # Publish current waypoint
        publish_waypoint(waypoints[waypoint_index])

        # Move to the next waypoint
        waypoint_index = (waypoint_index + 1) % len(waypoints)

        # Wait for the next loop
        rate.sleep()
