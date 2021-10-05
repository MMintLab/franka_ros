#!/usr/bin/env python

import rospy
import tf.transformations
import numpy as np

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

right_marker_pose = PoseStamped()
right_initial_pose_found = False
right_pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def rightPublisherCallback(msg, link_name):
    right_marker_pose.header.frame_id = link_name
    right_marker_pose.header.stamp = rospy.Time(0)
    right_pose_pub.publish(right_marker_pose)


def right_franka_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    right_marker_pose.pose.orientation.x = initial_quaternion[0]
    right_marker_pose.pose.orientation.y = initial_quaternion[1]
    right_marker_pose.pose.orientation.z = initial_quaternion[2]
    right_marker_pose.pose.orientation.w = initial_quaternion[3]
    right_marker_pose.pose.position.x = msg.O_T_EE[12]
    right_marker_pose.pose.position.y = msg.O_T_EE[13]
    right_marker_pose.pose.position.z = msg.O_T_EE[14]
    global right_initial_pose_found
    right_initial_pose_found = True


def rightProcessFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        right_marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        right_marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        right_marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        right_marker_pose.pose.orientation = feedback.pose.orientation
    right_server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("panda_2_equilibrium_pose_node")
    right_state_sub = rospy.Subscriber("/combined_panda/panda_2_state_controller/franka_states",
                                 FrankaState, right_franka_state_callback)

    listener = tf.TransformListener()
    right_link_name = "panda_2_link0"
    base_frame = "base"

    # Get initial pose for the interactive marker
    while not right_initial_pose_found:
        rospy.sleep(1)
    right_state_sub.unregister()

    right_pose_pub = rospy.Publisher(
        "panda_2_equilibrium_pose", PoseStamped, queue_size=10)
    right_server = InteractiveMarkerServer("panda_2_equilibrium_pose_marker")
    right_int_marker = InteractiveMarker()

    right_int_marker.header.frame_id = right_link_name
    right_int_marker.scale = 0.3
    right_int_marker.name = "panda_2_equilibrium_pose"
    right_int_marker.description = ("Rocket")
    right_int_marker.pose = right_marker_pose.pose
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: rightPublisherCallback(msg, right_link_name))

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    right_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    right_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    right_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    right_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    right_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    right_int_marker.controls.append(control)

    right_server.insert(right_int_marker, rightProcessFeedback)

    right_server.applyChanges()

    rospy.spin()
