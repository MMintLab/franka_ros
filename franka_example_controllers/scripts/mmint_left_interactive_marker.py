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

left_marker_pose = PoseStamped()
left_initial_pose_found = False
left_pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]


def leftPublisherCallback(msg, link_name):
    left_marker_pose.header.frame_id = link_name
    left_marker_pose.header.stamp = rospy.Time(0)
    left_pose_pub.publish(left_marker_pose)


def left_franka_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    left_marker_pose.pose.orientation.x = initial_quaternion[0]
    left_marker_pose.pose.orientation.y = initial_quaternion[1]
    left_marker_pose.pose.orientation.z = initial_quaternion[2]
    left_marker_pose.pose.orientation.w = initial_quaternion[3]
    left_marker_pose.pose.position.x = msg.O_T_EE[12]
    left_marker_pose.pose.position.y = msg.O_T_EE[13]
    left_marker_pose.pose.position.z = msg.O_T_EE[14]
    global left_initial_pose_found
    left_initial_pose_found = True


def leftProcessFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        left_marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        left_marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        left_marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        left_marker_pose.pose.orientation = feedback.pose.orientation
    left_server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("panda_1_equilibrium_pose_node")
    left_state_sub = rospy.Subscriber("panda_1_state_controller/franka_states",
                                 FrankaState, left_franka_state_callback)

    listener = tf.TransformListener()
    left_link_name = "panda_1_link0"
    base_frame = "base"

    # Get initial pose for the interactive marker
    while not left_initial_pose_found:
        rospy.sleep(1)
    left_state_sub.unregister()

    left_pose_pub = rospy.Publisher(
        "panda_1_equilibrium_pose", PoseStamped, queue_size=10)
    
    left_server = InteractiveMarkerServer("panda_1_equilibrium_pose_marker")
    left_int_marker = InteractiveMarker()

    left_int_marker.header.frame_id = left_link_name
    left_int_marker.scale = 0.3
    left_int_marker.name = "panda_1_equilibrium_pose"
    left_int_marker.description = ("Nebula")
    left_int_marker.pose = left_marker_pose.pose

    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: leftPublisherCallback(msg, left_link_name))
   
    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    left_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    left_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    left_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    left_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    left_int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    left_int_marker.controls.append(control)

    left_server.insert(left_int_marker, leftProcessFeedback)

    left_server.applyChanges()

    rospy.spin()
