import tf_transformations
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, Bool
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time

# from final_challenge2025.utils import map_to_robot_frame

class Controller:
    def __init__(self, ros_node):
        self.state_machine = ros_node
        self.banana_parking_controller = ros_node

        self.sign_state = -1.0

        # self.state_machine.declare_parameter("lookahead_distance", 0.5)
        self.state_machine.declare_parameter("path_request_topic", "/path_request")
        self.state_machine.declare_parameter("drive_topic", "/high_mux/ackermann_drive")
        # self.state_machine.declare_parameter("lookahead_point_topic", "/lookahead_point")
        # self.state_machine.declare_parameter("planned_path_topic", "/planned_path")
        
        # self.lookahead_distance = (
        #     self.state_machine.get_parameter("lookahead_distance")
        #     .get_parameter_value()
        #     .double_value
        # )

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
        self.path_request_publisher = self.state_machine.create_publisher(
            PoseArray, self.path_request_topic, 1
        )
        self.banana_parking_publisher = self.state_machine.create_publisher(
            Float32MultiArray, "/relative_banana_px", 1
        )

        self.banana_save_publisher = self.state_machine.create_publisher(
            Bool, "/banana_save_img", 1
        )

        self.drive_pub = self.state_machine.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1
        )
        self.state_machine.get_logger().info("Controller initialized")

    # def planned_path_callback(self, planned_path):
    #     """
    #     Callback for receiving the planned path.
        """
        # if not self.state_machine.waiting_for_path:
            # If we're not waiting for a path, ignore this callback
            # return
            
        # self.state_machine.get_logger().info("Received path plan, starting navigation")
        # self.current_path_plan = planned_path
        # self.current_path_index = 0
        # self.state_machine.waiting_for_path = False
        
        # Start the path execution
        # if self.state_machine.execute_path_timer is None:
        #     self.state_machine.execute_path_timer = self.state_machine.create_timer(0.01, self.execute_path)

    # def execute_path(self):
    #     """
    #     Execute one step of path following
    #     """
    #     if not self.current_path_plan or self.current_path_index >= len(self.current_path_plan.poses):
    #         self.current_path_plan = None
    #         self.current_path_index = 0
    #         if self.state_machine.execute_path_timer:
    #             self.state_machine.execute_path_timer.cancel()
    #             self.state_machine.execute_path_timer = None
    #         return True
            
    #     robot_pose = self.state_machine.current_pose
    #     if robot_pose is None:
    #         self.state_machine.get_logger().warn("No robot pose available")
    #         return False
            
    #     # Find lookahead point
    #     lookahead_point = self.find_lookahead_point()
        
    #     if lookahead_point is None:
    #         self.state_machine.get_logger().info("Reached end of path")
    #         self.current_path_plan = None
    #         self.current_path_index = 0
    #         if self.state_machine.execute_path_timer:
    #             self.state_machine.execute_path_timer.cancel()
    #             self.state_machine.execute_path_timer = None
    #         return True
            
    #     # Publish the lookahead point
    #     self.publish_lookahead_point(lookahead_point)
    #     return False

    def collect_banana(self):
        """
        Collect a banana.
        Collection involves stopping for 5 seconds.
        """
        self.state_machine.get_logger().info("Collecting banana")
        self.stop_car()
        # TODO: take a picture from the detector
        banana_sv_img = Bool()
        banana_sv_img.data = True
        self.banana_save_publisher.publish(banana_sv_img)
        time.sleep(5)
        backup_cmd = AckermannDriveStamped()
        backup_cmd.header.stamp = self.state_machine.get_clock().now().to_msg()
        backup_cmd.drive.steering_angle = 0.0
        backup_cmd.drive.speed = -1.0      
        self.drive_pub.publish(backup_cmd)
        time.sleep(2)
        self.stop_car()

    def sweep_banana(self):
        # backup_cmd = AckermannDriveStamped()
        # backup_cmd.header.stamp = self.state_machine.get_clock().now().to_msg()
        # backup_cmd.drive.steering_angle = -np.pi/6
        # backup_cmd.drive.speed = -1.0      
        # self.drive_pub.publish(backup_cmd)
        # # self.state_machine.get_logger().info(f'INITIAL CONTROL 1{backup_cmd.drive.steering_angle, backup_cmd.drive.speed}')
        # time.sleep(1)
        # turn_cmd = AckermannDriveStamped()
        # turn_cmd.header.stamp = self.state_machine.get_clock().now().to_msg()
        # turn_cmd.drive.steering_angle = np.pi/6
        # turn_cmd.drive.speed = 1.0
        # self.drive_pub.publish(turn_cmd)
        # # self.state_machine.get_logger().info(f'INITIAL CONTROL 2{turn_cmd.drive.steering_angle, turn_cmd.drive.speed}')
        # time.sleep(1)

        backup_cmd = AckermannDriveStamped()
        backup_cmd.header.stamp = self.state_machine.get_clock().now().to_msg()
        backup_cmd.drive.steering_angle = np.pi/6 * self.sign_state
        backup_cmd.drive.speed = 1.0 * self.sign_state
        self.drive_pub.publish(backup_cmd)
        self.sign_state = self.sign_state*-1
        # self.state_machine.get_logger().info(f'INITIAL CONTROL 1{backup_cmd.drive.steering_angle, backup_cmd.drive.speed}')
        time.sleep(1)

        self.stop_car()


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

    def banana_parking_phase(self, x, y):
        """
        Follow a path to a goal point.
        """
        # if self.state_machine.detection is not None:
        msg = Float32MultiArray()
        msg.data = [x, y]
        self.banana_parking_publisher.publish(msg)

        # self.banana_parking_publisher.publish()
        # if self.state_maching.detection is not None:
        #     self.banana_parking_controller.banana_parking
        # self.banana_parking_controller.banana_parking(self.state_machine.detection[0], self.state_machine.detection[1])
        # self.state_machine.get_logger().info("Parking near banana")

    def follow_path(self, start_pose, end_pose):
        """
        Follow a path to a goal point. 
        """
        # Only request a new path if we're not already following one
        # if not self.waiting_for_path and self.current_path_plan is None:
        #     self.state_machine.get_logger().info("Requesting path plan")
        #     path_request = PoseArray()
        #     path_request.poses = [start_pose, end_pose]
        #     self.waiting_for_path = True
        #     self.path_request_publisher.publish(path_request)
        self.state_machine.get_logger().info("Requesting path plan")
        path_request = PoseArray()
        path_request.poses = [start_pose, end_pose]
        self.path_request_publisher.publish(path_request)
        
    # def find_lookahead_point(self):
    #     """
    #     Find the lookahead point along the path.

    #     Args:
    #         robot_pose: Current robot pose in map frame

    #     Returns:
    #         Lookahead point in map frame or None if end of path reached
    #     """
    #     if self.current_path_index >= len(self.current_path_plan.poses) - 1:
    #         return None

    #     # Get current and next waypoint
    #     current_waypoint = self.current_path_plan.poses[self.current_path_index]
    #     next_waypoint = self.current_path_plan.poses[self.current_path_index + 1]

    #     # Robot position in map frame
    #     robot_x = self.state_machine.current_pose.position.x
    #     robot_y = self.state_machine.current_pose.position.y

    #     # Check if we've reached the current waypoint
    #     dx = current_waypoint.position.x - robot_x
    #     dy = current_waypoint.position.y - robot_y
    #     dist_to_current = np.sqrt(dx * dx + dy * dy)

    #     if (
    #         dist_to_current < self.lookahead_distance * 0.5
    #     ):  # TODO: multiply by 0.5 is arbitrary
    #         # Move to next waypoint
    #         self.current_path_index += 1
    #         if self.current_path_index >= len(self.current_path_plan.poses) - 1:
    #             return self.current_path_plan.poses[-1]  # Return final waypoint

    #         # Update current and next waypoint
    #         current_waypoint = self.current_path_plan.poses[self.current_path_index]
    #         next_waypoint = self.current_path_plan.poses[self.current_path_index + 1]

    #     # Calculate vector from current to next waypoint
    #     segment_x = next_waypoint.position.x - current_waypoint.position.x
    #     segment_y = next_waypoint.position.y - current_waypoint.position.y
    #     segment_length = np.sqrt(segment_x * segment_x + segment_y * segment_y)

    #     if segment_length < 1e-6:
    #         return next_waypoint  # Return next waypoint if segment is too short

    #     # Normalize the segment vector
    #     segment_x /= segment_length
    #     segment_y /= segment_length

    #     # Vector from robot to start of segment
    #     dx = current_waypoint.position.x - robot_x
    #     dy = current_waypoint.position.y - robot_y

    #     # Calculate closest point on line segment to robot
    #     proj_length = dx * segment_x + dy * segment_y
    #     closest_x = current_waypoint.position.x - proj_length * segment_x
    #     closest_y = current_waypoint.position.y - proj_length * segment_y

    #     # Distance from robot to closest point on line
    #     dx = closest_x - robot_x
    #     dy = closest_y - robot_y
    #     lateral_dist = np.sqrt(dx * dx + dy * dy)

    #     # If robot is far from path, project lookahead distance along line
    #     if lateral_dist > self.lookahead_distance:
    #         # Just move along the segment
    #         lookahead_x = closest_x + segment_x * self.lookahead_distance
    #         lookahead_y = closest_y + segment_y * self.lookahead_distance
    #     else:
    #         # Calculate coefficients for quadratic equation
    #         a = 1.0  # Since segment vector is normalized
    #         b = 2.0 * (segment_x * dx + segment_y * dy)
    #         c = dx * dx + dy * dy - self.lookahead_distance * self.lookahead_distance

    #         # Calculate discriminant
    #         discriminant = b * b - 4 * a * c

    #         # Calculate intersection points
    #         t1 = (-b + np.sqrt(discriminant)) / (2.0 * a)
    #         t2 = (-b - np.sqrt(discriminant)) / (2.0 * a)

    #         # Choose the intersection that is ahead of us (positive t)
    #         t = max(0.0, min(t1, t2))
    #         if t1 >= 0 and t2 >= 0:
    #             t = min(t1, t2)
    #         elif t1 >= 0:
    #             t = t1
    #         elif t2 >= 0:
    #             t = t2

    #         # Calculate lookahead point
    #         lookahead_x = closest_x + t * segment_x
    #         lookahead_y = closest_y + t * segment_y

    #     # Create the lookahead point with orientation from the segment
    #     lookahead_point = Pose()
    #     lookahead_point.position.x = lookahead_x
    #     lookahead_point.position.y = lookahead_y

    #     # Set orientation to point along the segment
    #     theta = np.arctan2(segment_y, segment_x)
    #     q = tf_transformations.quaternion_from_euler(0, 0, theta)
    #     lookahead_point.orientation.x = q[0]
    #     lookahead_point.orientation.y = q[1]
    #     lookahead_point.orientation.z = q[2]
    #     lookahead_point.orientation.w = q[3]

    #     return lookahead_point

    # def publish_lookahead_point(self, lookahead_point):
    #     """
    #     Transform lookahead point from map frame to robot frame and publish it.

    #     Args:
    #         robot_pose: Current robot pose in map frame
    #         lookahead_point: Lookahead point in map frame
    #     """
    #     lookahead_point_robot = map_to_robot_frame(lookahead_point, self.state_machine.current_pose)

    #     # Publish the lookahead point in robot frame
    #     msg = Float32MultiArray()
    #     msg.data = lookahead_point_robot
    #     self.lookahead_point_publisher.publish(msg)
