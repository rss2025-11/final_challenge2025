import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray

import numpy as np

from final_challenge2025.utils import SIGNAL_MAP_POSE
START_POSE = Pose(position=Point(x=-19.09, y=0.57, z=0.0))
BLANK_POSE = Pose(position=Point(x=0.0, y=0.0, z=0.0))

class BasementPointPublisher(Node):
    """
    Node that publishes a list of "shell" points
    Subscribes to the "Publish Point" topic when you click on the map in RViz
    After 3 points have been chosen, it publishes the 3 points as a PoseArray and resets the array
    """

    def __init__(self):
        super().__init__("BasementPointPub")
        self.publisher = self.create_publisher(PoseArray, "/shell_points", 1)
        self.goal_pose_subscriber = self.create_subscription(
            PointStamped, "/goal_pose", self.goal_pose_callback, 1
        )
        self.planned_path_subscriber = self.create_subscription(
            PoseArray, "/planned_path", self.planned_path_callback, 1
        )
        self.path_request_publisher = self.create_publisher(
            PoseArray, "/path_request", 1
        )

        self.goal_poses = []
        self.goal_distances = []

        self.initialized = False

        self.get_logger().info("Point Publisher Initialized")

    def goal_pose_callback(self, point_msg: PointStamped):
        x, y = point_msg.point.x, point_msg.point.y
        self.get_logger().info(f"Received point: {x}, {y}")
        self.goal_poses.append(Pose(position=Point(x=x, y=y, z=0.0)))
        path_request = PoseArray()
        path_request.poses = [START_POSE, self.goal_poses[-1]]
        self.path_request_publisher.publish(path_request)

        if len(self.goal_poses) == 2:
            # add in the signal location
            self.goal_poses.append(SIGNAL_MAP_POSE)

    def planned_path_callback(self, path_msg: PoseArray):
        if self.initialized:
            return
        
        self.get_logger().info(f"Received path: {path_msg}")
        total_distance = self.calculate_total_distance(path_msg)
        self.goal_distances.append(total_distance)

        if len(self.goal_distances) == 2:     
            # add in the signal location
            self.goal_poses.append(BLANK_POSE)
            path_request = PoseArray()
            path_request.poses = [START_POSE, SIGNAL_MAP_POSE]
            self.path_request_publisher.publish(path_request)
        
        if len(self.goal_distances) == 3:
            # sort the points by total distance
            sorted_goal_poses = [
                self.goal_poses[i] for i in np.argsort(self.goal_distances)
            ]
            self.publish(sorted_goal_poses)
            self.initialized = True


    def calculate_total_distance(self, path_msg: PoseArray):
        total_distance = 0
        for i in range(len(path_msg.poses) - 1):
            # calculate distance between each pair of points
            distance = np.sqrt(
                (path_msg.poses[i].position.x - path_msg.poses[i + 1].position.x) ** 2
                + (path_msg.poses[i].position.y - path_msg.poses[i + 1].position.y) ** 2
            )
            total_distance += distance
        return total_distance

    def publish(self, goal_poses: list[Pose]):
        # Publish PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.poses = goal_poses
        self.publisher.publish(pose_array)

        # Print to Command Line
        points_str = "\n" + "\n".join(
            [f"({p.position.x},{p.position.y})" for p in goal_poses]
        )
        self.get_logger().info(f"Published 3 points: {points_str}")

        # Reset arrays
        self.goal_poses = []
        self.goal_distances = []


def main(args=None):
    rclpy.init(args=args)
    node = BasementPointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
