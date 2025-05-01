import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class PurePursuit(Node):
    """Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed."""

    def __init__(self):
        super().__init__("pure_pursuit")
        self.declare_parameter("drive_topic", "/drive")

        self.drive_topic = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
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

        self.point_follow_pub = self.create_publisher(Marker, "/point_to_follow", 1)

        self.wheelbase_length = 0.33
        self.speed = 2.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.current_lookahead_point = [0.0, 0.0, 0.0]  # x, y, theta

    def speed_callback(self, msg):
        self.speed = msg.data

    def lookahead_point_callback(self, msg):
        self.viz_lookahead_point()
        self.current_lookahead_point = msg.data
        dx = self.current_lookahead_point[0]
        dy = self.current_lookahead_point[1]

        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = self.get_clock().now().to_msg()

        # find distance between robot and lookahead point
        lookahead = np.sqrt(dx**2 + dy**2)

        angle_to_goal = np.arctan2(dy, dx)
        angle = np.arctan(
            2 * self.wheelbase_length * np.sin(angle_to_goal) / (lookahead + 1e-6)
        )

        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = angle
        self.drive_pub.publish(drive_cmd)

    def viz_lookahead_point(self):
        """
        Visualize the lookahead point.
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.SPHERE
        marker.pose.position.x = self.current_lookahead_point[0]
        marker.pose.position.y = self.current_lookahead_point[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.point_follow_pub.publish(marker)
def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
