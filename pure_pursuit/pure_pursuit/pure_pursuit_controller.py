import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class PurePursuit(Node):
    """Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed."""

    def __init__(self):
        super().__init__("pure_pursuit")
        self.declare_parameter("odom_topic", "default")
        self.declare_parameter("drive_topic", "default")

        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.drive_topic = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )

        # Odometry topic will depend on task. For moon race, we may not use particle filter.
        # For shrink ray heist, we will definitely want to use localization
        self.pose_sub = self.create_subscription(
            Odometry, self.odom_topic, self.pose_callback, 1
        )

        self.speed_sub = self.create_subscription(
            Float32, "/speed", self.speed_callback, 1
        )

        # Gives the exact point that the racecar will navigate to
        # All computation of this point (whether from a trajectory or goal location)
        #   should be done separately
        # This should be in the robot frame
        self.lookahead_point = self.create_subscription(
            Float32MultiArray, "/lookahead_point", self.lookahead_point_callback, 1
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1
        )

        self.point_follow_pub = self.create_publisher(
            Marker, "/point_to_follow", 1
        )

        self.wheelbase_length = 0.33
        self.speed = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.current_lookahead_point = [0.0, 0.0, 0.0] # x, y, theta

    def lookahead_point_callback(self, msg):
        self.current_lookahead_point = msg.data

    def speed_callback(self, msg):
        self.speed = msg.data

    def pose_callback(self, odometry_msg):
        # Access the orientation quaternion
        orientation = odometry_msg.pose.pose.orientation
        # Convert quaternion to Euler angles
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_x = odometry_msg.pose.pose.position.x
        self.current_y = odometry_msg.pose.pose.position.y
        self.current_theta = yaw    

        self.control(self.current_lookahead_point)

    def control(self, lookahead_point):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()

        dx = lookahead_point[0]
        dy = lookahead_point[1]
        # find distance between robot and lookahead point
        lookahead = np.sqrt(dx**2 + dy**2)

        angle_to_goal = np.arctan2(dy, dx)
        angle = np.arctan(
            2 * self.wheelbase_length * np.sin(angle_to_goal) / (lookahead + 1e-6)
        )

        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = angle
        self.drive_pub.publish(drive_cmd)

def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
