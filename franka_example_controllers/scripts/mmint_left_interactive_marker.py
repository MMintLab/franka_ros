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

left_marker_pose = PoseStamped()
left_frame_ready = False
left_has_error = False
left_pose_pub = None
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


def publish_left_target_pose():
    """
    This function publishes left desired pose which the controller will subscribe to.
    :return: None
    """
    left_marker_pose.header.frame_id = "panda_1_link0"
    left_marker_pose.header.stamp = rospy.Time(0)
    left_pose_pub.publish(left_marker_pose)


def left_pose_callback(msg):
    """
    This callback function sets the left marker pose to the current left pose from a subscribed topic.
    :return: None
    """
    global left_frame_ready
    global left_marker_pose

    curr_quaternion = tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE, (4, 4))))
    curr_quaternion = curr_quaternion / np.linalg.norm(curr_quaternion)
    left_marker_pose.pose.orientation.x = curr_quaternion[0]
    left_marker_pose.pose.orientation.y = curr_quaternion[1]
    left_marker_pose.pose.orientation.z = curr_quaternion[2]
    left_marker_pose.pose.orientation.w = curr_quaternion[3]
    left_marker_pose.pose.position.x = msg.O_T_EE[12]
    left_marker_pose.pose.position.y = msg.O_T_EE[13]
    left_marker_pose.pose.position.z = msg.O_T_EE[14]
    left_frame_ready = True


def reset_left_marker_pose_blocking():
    """
    This function resets the marker pose to current left arm EE.
    :return: None
    """

    global left_frame_ready, left_marker_pose

    left_frame_ready = False

    left_frame_pose_sub = rospy.Subscriber(
        "/combined_panda/panda_1_state_controller/franka_states",
        PoseStamped, left_pose_callback)

    # Get initial pose for the interactive marker
    while not left_frame_ready:
        rospy.sleep(0.1)

    left_frame_pose_sub.unregister()


def left_state_callback(msg):
    global left_has_error
    if msg.robot_mode == FrankaState.ROBOT_MODE_MOVE:
        left_has_error = False
    else:
        left_has_error = True


def left_process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE and not left_has_error and not feedback.mouse_point_valid:
        left_marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        left_marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        left_marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        curr_quaternion = np.array([feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w])
        curr_quaternion = curr_quaternion / np.linalg.norm(curr_quaternion)
        left_marker_pose.pose.orientation.x = curr_quaternion[0]
        left_marker_pose.pose.orientation.y = curr_quaternion[1]
        left_marker_pose.pose.orientation.z = curr_quaternion[2]
        left_marker_pose.pose.orientation.w = curr_quaternion[3]
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE and not left_has_error and feedback.mouse_point_valid:
        reset_left_marker_pose_blocking()
        publish_left_target_pose()
        left_server.setPose("panda_1_equilibrium_pose", left_marker_pose.pose)
    left_server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("panda_1_equilibrium_pose_node")
    left_state_sub = rospy.Subscriber("/combined_panda/panda_1_state_controller/franka_states",
                                 FrankaState, left_state_callback)

    listener = tf.TransformListener()
    left_link_name = "panda_1_link0"

    # Get initial pose for the interactive marker
    reset_left_marker_pose_blocking()

    left_pose_pub = rospy.Publisher(
        "panda_1_equilibrium_pose", PoseStamped, queue_size=10)
    
    left_server = InteractiveMarkerServer("panda_1_equilibrium_pose_marker")
    left_int_marker = InteractiveMarker()

    left_int_marker.header.frame_id = left_link_name
    left_int_marker.scale = 0.3
    left_int_marker.name = "panda_1_equilibrium_pose"
    left_int_marker.description = ("Nebula")
    left_int_marker.pose = left_marker_pose.pose
   
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

    left_server.insert(left_int_marker, left_process_feedback)
    left_server.applyChanges()

    while not rospy.is_shutdown():
        publish_left_target_pose()
        if left_has_error:
            reset_left_marker_pose_blocking()
            publish_left_target_pose()
            left_server.setPose("panda_1_equilibrium_pose", left_marker_pose.pose)
            left_server.applyChanges()
            rospy.sleep(0.5)
        rospy.sleep(0.1)
