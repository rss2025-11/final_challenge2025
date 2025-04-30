import tf_transformations
import numpy as np

from geometry_msgs.msg import PoseArray, Float32MultiArray, Pose
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time


class Controller:
    def __init__(self, ros_node):
        self.state_machine = ros_node

        self.state_machine.declare_parameter("lookahead_distance", 0.5)
        self.state_machine.declare_parameter("path_request_topic", "/path_request")
        self.state_machine.declare_parameter("drive_topic", "/high_mux/ackermann_drive")
        self.state_machine.declare_parameter("lookahead_point_topic", "/lookahead_point")
        self.state_machine.declare_parameter("planned_path_topic", "/planned_path")
        
        self.lookahead_distance = (
            self.state_machine.get_parameter("lookahead_distance")
            .get_parameter_value()
            .float_value
        )

        self.drive_topic = (
            self.state_machine.get_parameter("drive_topic")
            .get_parameter_value()
            .string_value
        )
        self.path_request_topic = (
            self.state_machine.get_parameter("path_request_topic")
            .get_parameter_value()
            .string_value
        )
        self.lookahead_point_topic = (
            self.state_machine.get_parameter("lookahead_point_topic")
            .get_parameter_value()
            .string_value
        )
        self.planned_path_topic = (
            self.state_machine.get_parameter("planned_path_topic")
            .get_parameter_value()
            .string_value
        )

        self.lookahead_point_publisher = self.state_machine.create_publisher(
            Float32MultiArray,
            self.lookahead_point_topic,
            1,
        )

        self.path_request_publisher = self.state_machine.create_publisher(
            PoseArray, self.path_request_topic, 1
        )

        self.drive_pub = self.state_machine.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1
        )

        self.planned_path_subscriber = self.state_machine.create_subscription(
            PoseArray, self.planned_path_topic, self.planned_path_callback, 1
        )

        self.current_path_plan = None
        self.lookahead_distance = 0.5
        self.current_path_index = 0

    def planned_path_callback(self, planned_path):
        """
        Callback for receiving the planned path.
        """
        self.current_path_plan = planned_path

    def collect_banana(self, banana_location):
        """
        Collect a banana from the given location.
        Collection involves navigating to in front of the banana and then stopping for 5 seconds.
        """
        #  TODO: might want to route to in-front of banana, not on it
        self.follow_path(self.state_machine.current_pose, banana_location)
        time.sleep(5)

    def await_signal(self):
        """
        Await a signal by stopping and waiting for the signal to change.
        """
        # TODO: might not even need to pub a drive command, does sleep work?
        self.drive_pub.publish(
            AckermannDriveStamped(
                drive=AckermannDrive(
                    steering=0.0,
                    acceleration=0.0,
                    steering_angle=0.0,
                    steering_angle_velocity=0.0,
                )
            )
        )

    def follow_path(self, start_pose, end_pose):
        """
        Follow a path to a goal point.
        """
        path_request = PoseArray()
        path_request.poses = [start_pose, end_pose]
        self.path_request_publisher.publish(path_request)

        # Reset current path index
        self.current_path_index = 0

        while not self.current_path_plan:
            # wait for new path plan to populate
            time.sleep(0.1)

        # Get initial robot pose
        robot_pose = self.state_machine.current_pose

        # Loop until we reach the end of the path
        while self.current_path_index < len(self.current_path_plan.poses):
            # Update robot pose
            robot_pose = self.state_machine.current_pose
            if robot_pose is None:
                time.sleep(0.1)
                continue

            # Find lookahead point
            lookahead_point = self.find_lookahead_point(robot_pose)

            # If we've reached the end of the path
            if lookahead_point is None:
                break

            # Publish the lookahead point (transformed to robot frame)
            self.publish_lookahead_point(lookahead_point)

            # Small sleep to control update rate
            time.sleep(0.01)

        # TODO: might want to publish a stop command here
        self.current_path_index = 0
        self.current_path_plan = None

    def find_lookahead_point(self):
        """
        Find the lookahead point along the path.

        Args:
            robot_pose: Current robot pose in map frame

        Returns:
            Lookahead point in map frame or None if end of path reached
        """
        if self.current_path_index >= len(self.current_path_plan.poses) - 1:
            return None

        # Get current and next waypoint
        current_waypoint = self.current_path_plan.poses[self.current_path_index]
        next_waypoint = self.current_path_plan.poses[self.current_path_index + 1]

        # Robot position in map frame
        robot_x = self.state_machine.current_pose.position.x
        robot_y = self.state_machine.current_pose.position.y

        # Check if we've reached the current waypoint
        dx = current_waypoint.position.x - robot_x
        dy = current_waypoint.position.y - robot_y
        dist_to_current = np.sqrt(dx * dx + dy * dy)

        if (
            dist_to_current < self.lookahead_distance * 0.5
        ):  # TODO: multiply by 0.5 is arbitrary
            # Move to next waypoint
            self.current_path_index += 1
            if self.current_path_index >= len(self.current_path_plan.poses) - 1:
                return self.current_path_plan.poses[-1]  # Return final waypoint

            # Update current and next waypoint
            current_waypoint = self.current_path_plan.poses[self.current_path_index]
            next_waypoint = self.current_path_plan.poses[self.current_path_index + 1]

        # Calculate vector from current to next waypoint
        segment_x = next_waypoint.position.x - current_waypoint.position.x
        segment_y = next_waypoint.position.y - current_waypoint.position.y
        segment_length = np.sqrt(segment_x * segment_x + segment_y * segment_y)

        if segment_length < 1e-6:
            return next_waypoint  # Return next waypoint if segment is too short

        # Normalize the segment vector
        segment_x /= segment_length
        segment_y /= segment_length

        # Vector from robot to start of segment
        dx = current_waypoint.position.x - robot_x
        dy = current_waypoint.position.y - robot_y

        # Calculate closest point on line segment to robot
        proj_length = dx * segment_x + dy * segment_y
        closest_x = current_waypoint.position.x - proj_length * segment_x
        closest_y = current_waypoint.position.y - proj_length * segment_y

        # Distance from robot to closest point on line
        dx = closest_x - robot_x
        dy = closest_y - robot_y
        lateral_dist = np.sqrt(dx * dx + dy * dy)

        # If robot is far from path, project lookahead distance along line
        if lateral_dist > self.lookahead_distance:
            # Just move along the segment
            lookahead_x = closest_x + segment_x * self.lookahead_distance
            lookahead_y = closest_y + segment_y * self.lookahead_distance
        else:
            # Calculate coefficients for quadratic equation
            a = 1.0  # Since segment vector is normalized
            b = 2.0 * (segment_x * dx + segment_y * dy)
            c = dx * dx + dy * dy - self.lookahead_distance * self.lookahead_distance

            # Calculate discriminant
            discriminant = b * b - 4 * a * c

            # Calculate intersection points
            t1 = (-b + np.sqrt(discriminant)) / (2.0 * a)
            t2 = (-b - np.sqrt(discriminant)) / (2.0 * a)

            # Choose the intersection that is ahead of us (positive t)
            t = max(0.0, min(t1, t2))
            if t1 >= 0 and t2 >= 0:
                t = min(t1, t2)
            elif t1 >= 0:
                t = t1
            elif t2 >= 0:
                t = t2

            # Calculate lookahead point
            lookahead_x = closest_x + t * segment_x
            lookahead_y = closest_y + t * segment_y

        # Create the lookahead point with orientation from the segment
        lookahead_point = Pose()
        lookahead_point.position.x = lookahead_x
        lookahead_point.position.y = lookahead_y

        # Set orientation to point along the segment
        theta = np.arctan2(segment_y, segment_x)
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        lookahead_point.orientation.x = q[0]
        lookahead_point.orientation.y = q[1]
        lookahead_point.orientation.z = q[2]
        lookahead_point.orientation.w = q[3]

        return lookahead_point

    def publish_lookahead_point(self, lookahead_point):
        """
        Transform lookahead point from map frame to robot frame and publish it.

        Args:
            robot_pose: Current robot pose in map frame
            lookahead_point: Lookahead point in map frame
        """
        lookahead_point_robot = self.map_to_robot_frame(lookahead_point)

        # Publish the lookahead point in robot frame
        msg = Float32MultiArray()
        msg.data = lookahead_point_robot
        self.lookahead_point_publisher.publish(msg)

    def map_to_robot_frame(self, pose):
        """
        Transform pose from map frame to robot frame.

        Args:
            pose: Pose in map frame

        Returns:
            [x, y, theta] in robot frame
        """
        # Get robot position and orientation
        robot_x = self.state_machine.current_pose.position.x
        robot_y = self.state_machine.current_pose.position.y
        q = self.state_machine.current_pose.orientation
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

    def robot_to_map_frame(self, position) -> Pose:
        """
        Transform position from robot frame to map frame.

        Args:
            position: Position in robot frame, [x, y, theta]

        Returns:
            Pose in map frame
        """
        # Get robot position and orientation
        robot_x = self.state_machine.current_pose.position.x
        robot_y = self.state_machine.current_pose.position.y
        q = self.state_machine.current_pose.orientation
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
