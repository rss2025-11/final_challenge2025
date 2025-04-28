
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from hough_transform import edges_clean
from hough_transform import compute_lane_edges
class TrackSegment(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")

        # Subscribe to ZED camera RGB frames
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
        self.trajectory_pub = self.create_publisher(
            PoseArray , "/track_trajectory", 1
        )
             
    def image_callback(self, image_msg):

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        input_image = image
        hough_transformed, linesP = edges_clean(input_image)
        image_width = image.shape[1]
        lane_edges = self.compute_lane_edges(linesP, image_width)
        track_traj = self.compute_trajectory(lane_edges)
        self.trajectory_pub.publish(track_traj)


    def compute_trajectory(two_edges):
        # use 2 lines to determine trajectory (path between boundary lines)
        # determine the midpoint trajectory
        pass
def main(args=None):
    rclpy.init(args=args)
    cone_detector = TrackSegment()
    rclpy.spin(cone_detector)
    rclpy.shutdown()
