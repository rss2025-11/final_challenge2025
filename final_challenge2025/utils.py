import numpy as np
import tf_transformations

from geometry_msgs.msg import Pose

SIGNAL_MAP_POSE = Pose()
SIGNAL_MAP_POSE.position.x = -12.878
SIGNAL_MAP_POSE.position.y = 15.240
SIGNAL_MAP_POSE.position.z = 0.0
SIGNAL_MAP_POSE.orientation.x = 0.0
SIGNAL_MAP_POSE.orientation.y = 0.0
SIGNAL_MAP_POSE.orientation.z = 0.0
SIGNAL_MAP_POSE.orientation.w = 1.0

def map_to_robot_frame(pose, current_pose):
    """
    Transform pose from map frame to robot frame.

    Args:
        pose: Pose in map frame

    Returns:
        [x, y, theta] in robot frame
    """
    # Get robot position and orientation
    robot_x = current_pose.position.x
    robot_y = current_pose.position.y
    q = current_pose.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    _, _, robot_theta = tf_transformations.euler_from_quaternion(quaternion)

    # Calculate relative position (map frame to robot frame)
    dx = pose.position.x - robot_x
    dy = pose.position.y - robot_y

    # Rotate to robot frame
    cos_theta = np.cos(-robot_theta)
    sin_theta = np.sin(-robot_theta)

    # Transform to robot frame
    x_robot = dx * cos_theta - dy * sin_theta
    y_robot = dx * sin_theta + dy * cos_theta

    # Get lookahead point orientation in robot frame
    q = pose.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    _, _, lookahead_theta = tf_transformations.euler_from_quaternion(quaternion)
    theta_robot = lookahead_theta - robot_theta

    # Normalize angle
    theta_robot = np.arctan2(np.sin(theta_robot), np.cos(theta_robot))

    return [x_robot, y_robot, theta_robot]

def robot_to_map_frame(position, current_pose) -> Pose:
    """
    Transform position from robot frame to map frame.

    Args:
        position: Position in robot frame, [x, y, theta]

    Returns:
        Pose in map frame
    """
    # Get robot position and orientation
    robot_x = current_pose.position.x
    robot_y = current_pose.position.y
    q = current_pose.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    _, _, robot_theta = tf_transformations.euler_from_quaternion(quaternion)

    # Extract robot frame coordinates
    x_robot = position[0]
    y_robot = position[1]
    theta_robot = position[2]

    # Rotate by positive robot_theta (inverse of negative rotation)
    cos_theta = np.cos(robot_theta)
    sin_theta = np.sin(robot_theta)

    # Transform to map frame (inverse of robot frame transformation)
    x_map = x_robot * cos_theta - y_robot * sin_theta + robot_x
    y_map = x_robot * sin_theta + y_robot * cos_theta + robot_y

    # Add robot orientation to get map frame orientation
    theta_map = theta_robot + robot_theta

    # Normalize angle
    theta_map = np.arctan2(np.sin(theta_map), np.cos(theta_map))

    # Create pose in map frame
    pose = Pose()
    pose.position.x = x_map
    pose.position.y = y_map

    # Convert theta to quaternion
    q = tf_transformations.quaternion_from_euler(0, 0, theta_map)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose