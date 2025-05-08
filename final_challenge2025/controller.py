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
        self.state_machine.declare_parameter("path_planned_topic", "/planned_path")
        self.state_machine.declare_parameter("path_follow_topic", "/path_to_follow")
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
        self.path_planned_subscriber = self.state_machine.create_subscription(
            PoseArray, self.path_planned_topic, self.path_planned_callback, 1
        )
        self.path_follow_publisher = self.state_machine.create_publisher(
            PoseArray, self.path_follow_topic, 1
        )
        self.state_machine.get_logger().info("Controller initialized")

    def path_planned_callback(self, planned_path):
        """
        Callback for receiving the planned path.
        """
        if self.state_machine.current_phase == self.state_machine.Phase.FOLLOWING_PATH:
            self.path_follow_publisher.publish(planned_path)

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
