import tf_transformations
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, Bool
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time

class Controller:
    def __init__(self, ros_node):
        self.state_machine = ros_node
        self.banana_parking_controller = ros_node

        self.sign_state = -1.0

        self.state_machine.declare_parameter("path_request_topic", "/path_request")
        self.state_machine.declare_parameter("drive_topic", "/high_mux/ackermann_drive")

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

        backup_cmd = AckermannDriveStamped()
        backup_cmd.header.stamp = self.state_machine.get_clock().now().to_msg()
        backup_cmd.drive.steering_angle = np.pi/6 * self.sign_state
        sleep_time = 1.0
        if self.sign_state == -1:
            sleep_time = 1.3*sleep_time
        backup_cmd.drive.speed = 1.0 * self.sign_state
        self.drive_pub.publish(backup_cmd)
        self.sign_state = self.sign_state*-1
        time.sleep(sleep_time)

        self.stop_car()
        time.sleep(1)

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
        self.stop_car()

    def banana_parking_phase(self, x, y):
        """
        Follow a path to a goal point.
        """
        msg = Float32MultiArray()
        msg.data = [x, y]
        self.banana_parking_publisher.publish(msg)

    def follow_path(self, start_pose, end_pose):
        """
        Follow a path to a goal point. 
        """
        self.state_machine.get_logger().info("Requesting path plan")
        path_request = PoseArray()
        path_request.poses = [start_pose, end_pose]
        self.path_request_publisher.publish(path_request)