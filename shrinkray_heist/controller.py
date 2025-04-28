import tf_transformations

from geometry_msgs.msg import PoseArray, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import time


class Controller:
    def __init__(self, ros_node):
        self.node = ros_node

        self.node.declare_parameter("drive_topic", "/high_mux/ackermann_drive")
        self.drive_topic = (
            self.node.get_parameter("drive_topic").get_parameter_value().string_value
        )

        self.path_request_publisher = self.node.create_publisher(
            PoseArray, "/path_request", 1
        )
        self.lookahead_point_publisher = self.node.create_publisher(
            Float32MultiArray, "/lookahead_point", 1
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1
        )

        self.planned_path_subscriber = self.node.create_subscription(
            PoseArray, "/planned_path", self.planned_path_callback, 1
        )

    def planned_path_callback(self, planned_path):
        """
        Callback for receiving the planned path.
        """
        for pose in planned_path.poses:
            q = pose.orientation
            quaternion = [q.x, q.y, q.z, q.w]
            _, _, theta = tf_transformations.euler_from_quaternion(quaternion)
            self.lookahead_point_publisher.publish(
                [pose.position.x, pose.position.y, theta]
            )

    def collect_banana(self, banana_location):
        """
        Collect a banana from the given location.
        Collection involves navigating to in front of the banana and then stopping for 5 seconds.
        """
        q = banana_location.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, theta = tf_transformations.euler_from_quaternion(quaternion)
        self.lookahead_point_publisher.publish(
            [banana_location.pose.position.x, banana_location.pose.position.y, theta]
        )

    def avoid_human(self):
        """
        Avoid a human by navigating around them.
        """
        # TODO: determine break condition
        while True:
            # TODO: identify human obstacle

            # TODO: request path around human obstacle

            # TODO: follow path around human obstacle

            return

    def await_signal(self):
        """
        Await a signal by stopping and waiting for the signal to change.
        """
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
        time.sleep(
            1 / 60
        )  # make this control 60 Hz, presumably faster than lower mux control

    def follow_path(self, start_pose, end_pose):
        """
        Follow a path to a goal point.
        """
        path_request = PoseArray()
        path_request.poses = [start_pose, end_pose]
        self.path_request_publisher.publish(path_request)

        # TODO: need to wait for path to be planned and then follow it via the pure pursuit controller
