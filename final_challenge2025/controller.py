import tf_transformations
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time

from final_challenge2025.utils import map_to_robot_frame

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
            .double_value
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
        self.waiting_for_path = False  # Flag to track if we're waiting for a path

        self.state_machine.get_logger().info("Controller initialized")

    def planned_path_callback(self, planned_path):
        """
        Callback for receiving the planned path.
        """
        if not self.waiting_for_path:
            # If we're not waiting for a path, ignore this callback
            return
            
        self.state_machine.get_logger().info("Received path plan, starting navigation")
        self.current_path_plan = planned_path
        self.current_path_index = 0
        self.waiting_for_path = False
        
        # Start the path execution
        if self.state_machine.execute_path_timer is None:
            self.state_machine.execute_path_timer = self.state_machine.create_timer(0.01, self.execute_path)

    def execute_path(self):
        """
        Execute one step of path following
        """
        if not self.current_path_plan or self.current_path_index >= len(self.current_path_plan.poses):
            self.current_path_plan = None
            self.current_path_index = 0
            if self.state_machine.execute_path_timer:
                self.state_machine.execute_path_timer.cancel()
                self.state_machine.execute_path_timer = None
            return True
            
        robot_pose = self.state_machine.current_pose
        if robot_pose is None:
            self.state_machine.get_logger().warn("No robot pose available")
            return False
            
        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        
        if lookahead_point is None:
            self.state_machine.get_logger().info("Reached end of path")
            self.current_path_plan = None
            self.current_path_index = 0
            if self.state_machine.execute_path_timer:
                self.state_machine.execute_path_timer.cancel()
                self.state_machine.execute_path_timer = None
            return True
            
        # Publish the lookahead point
        self.publish_lookahead_point(lookahead_point)
        return False

    def collect_banana(self, banana_location):
        """
        Collect a banana from the given location.
        Collection involves stopping for 5 seconds.
        """
        self.state_machine.get_logger().info("Collecting banana")
        self.stop_car()
        time.sleep(5)

    def stop_car(self):
        """
        Stop the car by publishing a zero drive command.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive = AckermannDrive(
            speed=0.0,
            acceleration=0.0,
            jerk=0.0,
            steering_angle=0.0,
            steering_angle_velocity=0.0
        )
        self.drive_pub.publish(drive_msg)

    def await_signal(self):
        """
        Await a signal by stopping and waiting for the signal to change.
        """
        # TODO: might not even need to pub a drive command, does sleep work?
        self.stop_car()
        time.sleep(5)

    def follow_path(self, start_pose, end_pose):
        """
        Follow a path to a goal point.
        """
        # Only request a new path if we're not already following one
        if not self.waiting_for_path and self.current_path_plan is None:
            self.state_machine.get_logger().info("Requesting path plan")
            path_request = PoseArray()
            path_request.poses = [start_pose, end_pose]
            self.waiting_for_path = True
            self.path_request_publisher.publish(path_request)
        
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

        # Get next waypoint
        next_waypoint = self.current_path_plan.poses[self.current_path_index + 1]

        # Robot position in map frame
        robot_x = self.state_machine.current_pose.position.x
        robot_y = self.state_machine.current_pose.position.y

        # Check if we've reached the current waypoint
        dx = next_waypoint.position.x - robot_x
        dy = next_waypoint.position.y - robot_y
        dist_to_next = np.sqrt(dx * dx + dy * dy)

        if dist_to_next < self.lookahead_distance * 0.5:
            # Move to next waypoint
            self.current_path_index += 1
            if self.current_path_index >= len(self.current_path_plan.poses) - 1:
                return self.current_path_plan.poses[-1]  # Return final waypoint

            # Update next waypoint
            next_waypoint = self.current_path_plan.poses[self.current_path_index + 1]
            dx = next_waypoint.position.x - robot_x
            dy = next_waypoint.position.y - robot_y
            dist_to_next = np.sqrt(dx * dx + dy * dy)

        # Calculate vector from robot position to next waypoint
        segment_x = dx  # Next waypoint x - robot x
        segment_y = dy  # Next waypoint y - robot y
        segment_length = dist_to_next

        if segment_length < 1e-6:
            return next_waypoint  # Return next waypoint if segment is too short

        # Normalize the segment vector
        segment_x /= segment_length
        segment_y /= segment_length

        # For simplicity, always set lookahead point along the direct path to the next waypoint
        # at the lookahead distance or the distance to the next waypoint, whichever is smaller
        lookahead_dist = min(self.lookahead_distance, dist_to_next)
        
        # Calculate lookahead point
        lookahead_x = robot_x + segment_x * lookahead_dist
        lookahead_y = robot_y + segment_y * lookahead_dist

        # Create the lookahead point with orientation pointing toward the next waypoint
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
        lookahead_point_robot = map_to_robot_frame(lookahead_point, self.state_machine.current_pose)

        # Publish the lookahead point in robot frame
        msg = Float32MultiArray()
        msg.data = lookahead_point_robot
        self.lookahead_point_publisher.publish(msg)
