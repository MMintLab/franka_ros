#!/usr/bin/env python
""" This simple script creates an interactive marker for changing desired centering pose of
    two the dual_panda_cartesian_impedance_example_controller. It features also resetting the
    marker to current centering pose between the left and the right endeffector.
"""

import rospy
import argparse

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl, Marker
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped

left_marker_pose = PoseStamped()
right_marker_pose = PoseStamped()

left_frame_ready = False
right_frame_ready = False

has_error = False
left_has_error = False
right_has_error = False

left_pose_pub = None
right_pose_pub = None

left_position_limits = [[-0.1, 0.0], [0.45, 0.55], [1.05, 1.15]]
right_position_limits = [[-0.05, 0.05], [-0.05, 0.05], [1.0, 1.1]]

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
    left_marker_pose.header.frame_id = "base"
    left_marker_pose.header.stamp = rospy.Time(0)
    left_pose_pub.publish(left_marker_pose)

def publish_right_target_pose():
    """
    This function publishes right desired pose which the controller will subscribe to.
    :return: None
    """
    right_marker_pose.header.frame_id = "base"
    right_marker_pose.header.stamp = rospy.Time(0)
    right_pose_pub.publish(right_marker_pose)


def left_franka_state_callback(msg):
    """
    This callback function set `has_error` variable to True if the left arm is having an error.
    :param msg: FrankaState msg data
    :return:  None
    """
    global has_error, left_has_error
    if msg.robot_mode == FrankaState.ROBOT_MODE_MOVE:
        left_has_error = False
    else:
        left_has_error = True
    has_error = left_has_error or right_has_error


def right_franka_state_callback(msg):
    """
    This callback function set `has_error` variable to True if the right arm is having an error.
    :param msg: FrankaState msg data
    :return:  None
    """
    global has_error, right_has_error
    if msg.robot_mode == FrankaState.ROBOT_MODE_MOVE:
        right_has_error = False
    else:
        right_has_error = True
    has_error = left_has_error or right_has_error


def left_pose_callback(msg):
    """
    This callback function sets the left marker pose to the current left pose from a subscribed topic.
    :return: None
    """
    global left_frame_ready
    global left_marker_pose

    left_marker_pose = msg
    left_frame_ready = True

def right_pose_callback(msg):
    """
    This callback function sets the right marker pose to the current right pose from a subscribed topic.
    :return: None
    """
    global right_frame_ready
    global right_marker_pose

    right_marker_pose = msg
    right_frame_ready = True

def reset_marker_pose_blocking():
    """
    This function resets the marker pose to current left and right arm EEs.
    :return: None
    """

    global left_frame_ready, right_frame_ready
    global left_marker_pose, right_marker_pose

    left_frame_ready = False
    right_frame_ready = False

    left_frame_pose_sub = rospy.Subscriber(
        "mmint_dual_arm_cartesian_impedance_controller/left_frame",
        PoseStamped, left_pose_callback)

    right_frame_pose_sub = rospy.Subscriber(
        "mmint_dual_arm_cartesian_impedance_controller/right_frame",
        PoseStamped, right_pose_callback)

    # Get initial pose for the interactive marker
    while not (left_frame_ready and right_frame_ready):
        rospy.sleep(0.1)

    left_frame_pose_sub.unregister()
    right_frame_pose_sub.unregister()

# TODO: clip into predefined box (not done here? but done in the single arm version?)
# TODO: check to see which frame we're expecting msg to be in 

def process_left_feedback(left_feedback):
    """
    This callback function clips the left_marker_pose inside a predefined box to prevent misuse of the marker.
    :param feedback: feedback data of interactive marker
    :return: None
    """
    global left_marker_pose
    if left_feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        left_marker_pose.pose.position.x = max([min([left_feedback.pose.position.x,
                                                left_position_limits[0][1]]),
                                                left_position_limits[0][0]])
        left_marker_pose.pose.position.y = max([min([left_feedback.pose.position.y,
                                                left_position_limits[1][1]]),
                                                left_position_limits[1][0]])
        left_marker_pose.pose.position.z = max([min([left_feedback.pose.position.z,
                                                left_position_limits[2][1]]),
                                                left_position_limits[2][0]])
        left_marker_pose.pose.orientation = left_feedback.pose.orientation
    left_server.applyChanges()

def process_right_feedback(right_feedback):
    """
    This callback function clips the right_marker_pose inside a predefined box to prevent misuse of the marker.
    :param feedback: feedback data of interactive marker
    :return: None
    """
    global right_marker_pose
    if right_feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        right_marker_pose.pose.position.x = max([min([right_feedback.pose.position.x,
                                                right_position_limits[0][1]]),
                                                right_position_limits[0][0]])
        right_marker_pose.pose.position.y = max([min([right_feedback.pose.position.y,
                                                right_position_limits[1][1]]),
                                                right_position_limits[1][0]])
        right_marker_pose.pose.position.z = max([min([right_feedback.pose.position.z,
                                                right_position_limits[2][1]]),
                                                right_position_limits[2][0]])
        right_marker_pose.pose.orientation = right_feedback.pose.orientation
    right_server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("target_pose_node")

    parser = argparse.ArgumentParser("mmint_dual_arm_interactive_marker.py")
    parser.add_argument("--left_arm_id",
                        help="The id of the left arm.",
                        required=True)
    parser.add_argument("--right_arm_id",
                        help="The id of the right arm.",
                        required=True)
    parser.add_argument('args', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    # Arm IDs of left and right arms
    left_arm_id = args.left_arm_id
    right_arm_id = args.right_arm_id

    # Initialize subscribers for error states of the arms
    left_state_sub = rospy.Subscriber(left_arm_id + "_state_controller/franka_states",
                                 FrankaState, left_franka_state_callback)

    right_state_sub = rospy.Subscriber(right_arm_id + "_state_controller/franka_states",
                                 FrankaState, right_franka_state_callback)

    # Set marker pose to be the current "middle pose" of both EEs
    reset_marker_pose_blocking()

    # Initialize publisher for publishing desired centering pose
    left_pose_pub = rospy.Publisher(
        "mmint_dual_arm_cartesian_impedance_controller/left_frame_target_pose",
        PoseStamped,
        queue_size=1)
    right_pose_pub = rospy.Publisher(
        "mmint_dual_arm_cartesian_impedance_controller/right_frame_target_pose",
        PoseStamped,
        queue_size=1)

    # Interactive marker settings
    left_server = InteractiveMarkerServer("left_target_pose_marker")
    left_int_marker = InteractiveMarker()
    left_int_marker.header.frame_id = left_marker_pose.header.frame_id
    left_int_marker.scale = 0.3
    left_int_marker.name = "left_frame_pose"
    left_int_marker.description = ("nebula_EE")
    left_int_marker.pose = left_marker_pose.pose

    right_server = InteractiveMarkerServer("right_target_pose_marker")
    right_int_marker = InteractiveMarker()
    right_int_marker.header.frame_id = right_marker_pose.header.frame_id
    right_int_marker.scale = 0.3
    right_int_marker.name = "right_frame_pose"
    right_int_marker.description = ("rocket_EE")
    right_int_marker.pose = right_marker_pose.pose

    # insert a box
    control_xr = InteractiveMarkerControl()
    control_xr.orientation.w = 1
    control_xr.orientation.x = 1
    control_xr.orientation.y = 0
    control_xr.orientation.z = 0
    control_xr.name = "rotate_x"
    control_xr.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    left_int_marker.controls.append(control_xr)
    right_int_marker.controls.append(control_xr)

    control_xt = InteractiveMarkerControl()
    control_xt.orientation.w = 1
    control_xt.orientation.x = 1
    control_xt.orientation.y = 0
    control_xt.orientation.z = 0
    control_xt.name = "move_x"
    control_xt.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    left_int_marker.controls.append(control_xt)
    right_int_marker.controls.append(control_xt)

    control_yr = InteractiveMarkerControl()
    control_yr.orientation.w = 1
    control_yr.orientation.x = 0
    control_yr.orientation.y = 0
    control_yr.orientation.z = 1
    control_yr.name = "rotate_y"
    control_yr.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    left_int_marker.controls.append(control_yr)
    right_int_marker.controls.append(control_yr)

    control_yt = InteractiveMarkerControl()
    control_yt.orientation.w = 1
    control_yt.orientation.x = 0
    control_yt.orientation.y = 0
    control_yt.orientation.z = 1
    control_yt.name = "move_y"
    control_yt.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    left_int_marker.controls.append(control_yt)
    right_int_marker.controls.append(control_yt)

    control_zr = InteractiveMarkerControl()
    control_zr.orientation.w = 1
    control_zr.orientation.x = 0
    control_zr.orientation.y = 1
    control_zr.orientation.z = 0
    control_zr.name = "rotate_z"
    control_zr.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    left_int_marker.controls.append(control_zr)
    right_int_marker.controls.append(control_zr)

    control_zt = InteractiveMarkerControl()
    control_zt.orientation.w = 1
    control_zt.orientation.x = 0
    control_zt.orientation.y = 1
    control_zt.orientation.z = 0
    control_zt.name = "move_z"
    control_zt.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    left_int_marker.controls.append(control_zt)
    right_int_marker.controls.append(control_zt)

    control_3D = InteractiveMarkerControl()
    control_3D.orientation.w = 1
    control_3D.orientation.x = 1
    control_3D.orientation.y = 1
    control_3D.orientation.z = 1
    control_3D.name = "move_3D"
    control_3D.always_visible = True
    control_3D.markers.append(make_sphere())
    control_3D.interaction_mode = InteractiveMarkerControl.MOVE_3D
    left_int_marker.controls.append(control_3D)
    right_int_marker.controls.append(control_3D)

    # NOTE: Y and Z are switched, don't know why

    left_server.insert(left_int_marker, process_left_feedback)
    left_server.applyChanges()

    right_server.insert(right_int_marker, process_right_feedback)
    right_server.applyChanges()

    # main loop
    while not rospy.is_shutdown():
        publish_left_target_pose()
        publish_right_target_pose()
        if has_error:
            reset_marker_pose_blocking()
            publish_left_target_pose()
            publish_right_target_pose()
            left_server.setPose("left_frame_pose", left_marker_pose.pose)
            right_server.setPose("right_frame_pose", right_marker_pose.pose)
            left_server.applyChanges()
            right_server.applyChanges()
            rospy.sleep(0.5)
        rospy.sleep(0.1)
