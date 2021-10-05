#!/usr/bin/env python

import rospy
import tf.transformations
import numpy as np

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

right_marker_pose = PoseStamped()
right_frame_ready = False
right_has_error = False
right_pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

def make_sphere(scale=0.3):
    """
    This function returns sphere marker for 3D translational movements.
    :param scale: scales the size of the sphere
    :return: sphere marker
    """
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = scale * 0.45
    marker.scale.y = scale * 0.45
    marker.scale.z = scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker


def publish_right_target_pose():
    """
    This function publishes right desired pose which the controller will subscribe to.
    :return: None
    """
    right_marker_pose.header.frame_id = "panda_2_link0"
    right_marker_pose.header.stamp = rospy.Time(0)
    right_pose_pub.publish(right_marker_pose)


def right_pose_callback(msg):
    """
    This callback function sets the right marker pose to the current right pose from a subscribed topic.
    :return: None
    """
    global right_frame_ready
    global right_marker_pose

    curr_quaternion = tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    curr_quaternion = curr_quaternion / np.linalg.norm(curr_quaternion)
    right_marker_pose.pose.orientation.x = curr_quaternion[0]
    right_marker_pose.pose.orientation.y = curr_quaternion[1]
    right_marker_pose.pose.orientation.z = curr_quaternion[2]
    right_marker_pose.pose.orientation.w = curr_quaternion[3]
    right_marker_pose.pose.position.x = msg.O_T_EE[12]
    right_marker_pose.pose.position.y = msg.O_T_EE[13]
    right_marker_pose.pose.position.z = msg.O_T_EE[14]
    right_frame_ready = True


def reset_right_marker_pose_blocking():
    """
    This function resets the marker pose to current right arm EE.
    :return: None
    """

    global right_frame_ready, right_marker_pose

    right_frame_ready = False

    right_frame_pose_sub = rospy.Subscriber(
        "/combined_panda/panda_2_state_controller/franka_states",
        PoseStamped, right_pose_callback)

    # Get initial pose for the interactive marker
    while not right_frame_ready:
        rospy.sleep(0.1)

    right_frame_pose_sub.unregister()


def right_state_callback(msg):
    global right_has_error
    if msg.robot_mode == FrankaState.ROBOT_MODE_MOVE:
        right_has_error = False
    else:
        right_has_error = True


def right_process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE and not right_has_error:
        right_marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        right_marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        right_marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        curr_quaternion = np.array([feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w])
        curr_quaternion = curr_quaternion / np.linalg.norm(curr_quaternion)
        right_marker_pose.pose.orientation.x = curr_quaternion[0]
        right_marker_pose.pose.orientation.y = curr_quaternion[1]
        right_marker_pose.pose.orientation.z = curr_quaternion[2]
        right_marker_pose.pose.orientation.w = curr_quaternion[3]
    right_server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("panda_2_equilibrium_pose_node")
    right_state_sub = rospy.Subscriber("/combined_panda/panda_2_state_controller/franka_states",
                                 FrankaState, right_state_callback)

    listener = tf.TransformListener()
    right_link_name = "panda_2_link0"

    # Get initial pose for the interactive marker
    reset_right_marker_pose_blocking()

    right_pose_pub = rospy.Publisher(
        "panda_2_equilibrium_pose", PoseStamped, queue_size=10)
    
    right_server = InteractiveMarkerServer("panda_2_equilibrium_pose_marker")
    right_int_marker = InteractiveMarker()

    right_int_marker.header.frame_id = right_link_name
    right_int_marker.scale = 0.3
    right_int_marker.name = "panda_2_equilibrium_pose"
    right_int_marker.description = ("Rocket")
    right_int_marker.pose = right_marker_pose.pose
   
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

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 1
    control.orientation.z = 1
    control.name = "move_3D"
    control.always_visible = True
    control.markers.append(make_sphere())
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    right_int_marker.controls.append(control)

    right_server.insert(right_int_marker, right_process_feedback)
    right_server.applyChanges()

    while not rospy.is_shutdown():
        publish_right_target_pose()
        if right_has_error:
            reset_right_marker_pose_blocking()
            publish_right_target_pose()
            right_server.setPose("panda_2_equilibrium_pose", right_marker_pose.pose)
            right_server.applyChanges()
            rospy.sleep(0.5)
        rospy.sleep(0.1)
