
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from final_challenge2025.track_segmentation.track_segmentation.homography_matrix import *

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
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.bridge = CvBridge()
        self.homography = create_homography_matrix()
        self.inv_homography = np.linalg.inv(self.homography)
        self.lookahead_dist = 2.0 # meters
        self.lookahead_pt_history = np.tile([self.lookahead_dist, 0.0], (20, 1))

    def image_callback(self, image_msg):
        try:
            # 1. Convert ROS image to OpenCV
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            height, width = image.shape[:2]
            display_frame = image.copy()

            # 2. Process edges
            cdstP, linesP = self.edges_clean(image)

            if linesP is not None:
                # 3. Detect closest line and project to ground plane
                closest_line, is_left = self.compute_lane_edges(
                    linesP, width, height, display_frame)
                point1 = self.transform_homography(self.homography, closest_line[0])
                point2 = self.transform_homography(self.homography, closest_line[1])

                # 4. Get lookahead and smooth it
                lookahead_pt = self.determine_lookahead_point(
                    point1, point2, is_left)
                weighted_lookahead = self.compute_weighted_lookahead(
                    self.lookahead_pt_history, lookahead_pt)

                # 5. Project lookahead point to image space
                pixel_x, pixel_y = self.transform_homography(self.inv_homography, weighted_lookahead)
                cv.circle(image, (int(round(pixel_x)), int(round(pixel_y))), radius=5, color=(0, 255, 0), thickness=-1)
                debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().info("Error in image_callback:", e)


    def skeletonize(self, image):
        # Ensure the image is grayscale
        if len(image.shape) == 3:
            image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        
        # Ensure image is binary (black and white)
        _, binary = cv.threshold(image, 127, 255, cv.THRESH_BINARY)
        # Skeletonize using OpenCV's thinning function
        skeleton = np.zeros_like(binary)
        skeleton = cv.ximgproc.thinning(binary, skeleton, thinningType = cv.ximgproc.THINNING_GUOHALL)
        return skeleton  # Return the skeletonized image
    
        
    def edges_clean(self, image):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # Define white color range in HSV
        lower_white = np.array([0, 0, 150], dtype=np.uint8)
        upper_white = np.array([180, 25, 255], dtype=np.uint8)
        # Create mask for white color
        hsv[:np.shape(hsv)[0]//3,:] = 255
        mask_initial = cv.inRange(hsv, lower_white, upper_white)
        _, mask = cv.threshold(mask_initial, 127, 255, cv.THRESH_BINARY)

        # Apply erosion to thin thick lines slightly
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (1, 1))
        thinned_mask = self.skeletonize(mask)

        # Convert to BGR for visualization
        linesP = cv.HoughLinesP(thinned_mask, 1, np.pi / 180, 50, None, 100, 20) # TODO: Tune these numbers if needed

        # # For visualization only:
        cdstP_vis = cv.cvtColor(thinned_mask, cv.COLOR_GRAY2BGR)
        return cdstP_vis, linesP
    
    def compute_lane_edges(self, lines, image_width, image_height):
        # lines shape: (N, 1, 4), where 4 = (x1, y1, x2, y2)
        lines = np.squeeze(lines)  # (N, 4)

        # Compute horizontal spread (|x2 - x1|) for all lines
        x1 = lines[:, 0]
        x2 = lines[:, 2]
        y1 = lines[:, 1]
        y2 = lines[:, 3]

        # TODO: how should we handle case where y2-y1 = 0?
        # Should replace that intercept with inf

        # if np.any(abs(x1-x2) < 0.001):
        #     intercept_to_robot = np.avg(x1)
        # else:
        dx = x1 - x2
        dy = y1 - y2
        # Create mask for non-horizontal lines (dy >= 5)
        non_horizontal_mask = (abs(dy) >= 5)
        # Filter out horizontal lines
        lines = lines[non_horizontal_mask]
        dx = dx[non_horizontal_mask]
        dy = dy[non_horizontal_mask]
        x1 = x1[non_horizontal_mask]
        y1 = y1[non_horizontal_mask]
        y2 = y2[non_horizontal_mask]

        non_vertical_mask = (abs(dx) >= 5)

        lines = lines[non_vertical_mask]
        dx = dx[non_vertical_mask]
        dy = dy[non_vertical_mask]
        x1 = x1[non_vertical_mask]
        y1 = y1[non_vertical_mask]
        y2 = y2[non_vertical_mask]

        slope = np.divide(dy, dx)

        vertical_enough_mask = (abs(slope) > 0.2)

        lines = lines[vertical_enough_mask]
        dx = dx[vertical_enough_mask]
        dy = dy[vertical_enough_mask]
        x1 = x1[vertical_enough_mask]
        y1 = y1[vertical_enough_mask]
        y2 = y2[vertical_enough_mask]
        slope = slope[vertical_enough_mask]


        # intercept_to_robot describes the x intercept at the bottom of the image
        # i.e. where vertically does the line reach the camera?
        intercept_to_robot = (np.divide(image_height - y1, slope) + x1)
        score = np.abs(image_width/2 - intercept_to_robot)
        max_index = np.argmin(score)

        if (intercept_to_robot[max_index] < image_width/2):
            is_left = 1
        else:
            is_left = 0

        closest_line = lines[max_index]
        closest_line = np.array(([closest_line[0], closest_line[1]], [closest_line[2], closest_line[3]]))
        return closest_line, is_left  # shape (2, 4)
    
    def transform_homography(self, point):
        u = point[0]
        v = point[1]
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.homography, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]

        return [x, y]
    
    def determine_lookahead_point(self, point1, point2, is_left):
        # assume the points have been transformed into robot frame using homography matrix already
        p1x = point1[0]
        p1y = point1[1]
        p2x = point2[0]
        p2y = point2[1]
        slope = (p2y - p1y) / (p2x - p1x)
        y_intercept = slope * (self.lookahead_dist - p1x) + p1y
        if is_left:
            offset = -0.25
        else:
            offset = 0.25
        lookahead_point = [self.lookahead_dist, y_intercept + offset]
        return lookahead_point

    def compute_weighted_lookahead(self, new_lookahead_point):

        # TODO: if the most recently computed point is very far from the rest, don't use the raw calculation

        self.lookahead_pt_history = np.roll(self.lookahead_pt_history, 1, axis=0)
        self.lookahead_pt_history[0] = new_lookahead_point

        weights = np.ones(20) * (0.4 / 17)  # Remaining 17 entries share 0.4
        weights[0] = 0.3  # Newest
        weights[1] = 0.2  # 2nd newest
        weights[2] = 0.1  # 3rd newest

        weighted_point = np.sum(self.lookahead_pt_history * weights[:, np.newaxis], axis=0)
        return weighted_point
    
def main(args=None):
    rclpy.init(args=args)
    cone_detector = TrackSegment()
    rclpy.spin(cone_detector)
    rclpy.shutdown()
