#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

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
        self.debug_pub = self.create_publisher(Image, "/debug_img", 10)
        self.speed_pub = self.create_publisher(Float32, "/speed", 10)
        self.lookahead_pub = self.create_publisher(Float32MultiArray, "/lookahead_point", 10)
        self.lookahead_pixel = self.create_publisher(Float32MultiArray, "/lookahead_pixel", 10)
        self.bridge = CvBridge()
        self.homography = self.create_homography_matrix()
        self.inv_homography = np.linalg.inv(self.homography)
        self.lookahead_dist = 3.0 #2.0 # meters
        self.lookahead_pt_history = np.tile([self.lookahead_dist, 0.0], (10, 1))

    
    def create_homography_matrix(self):
        # PTS_IMAGE_PLANE = [[544, 257], [155, 207], [304, 196], [545, 193], [237, 177], [269, 196], [398, 168]]  
        PTS_IMAGE_PLANE = [[408, 333], [287, 215], [214, 219], [237, 190], [376, 192], [462, 231], [469, 203]]  
        # PTS_GROUND_PLANE = [[26.25, -14.5], [44.75, 21], [61, 2.25], [96.75, 17.75], [58.5, -38.25] ,[120.75,12.25], [123, -31.75]]  # dummy points
        PTS_GROUND_PLANE = [[37.5, -3.5], [107, 15], [100, 35.5], [174, 26.75], [164.5, -22.5] ,[87, -29.5], [134.5, -51.5]]  # dummy points
        
        METERS_PER_CM = 0.01

        METERS_PER_INCH = 0.0254

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            print(
                "ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length"
            )

        # Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_CM
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])
        homography, err = cv.findHomography(np_pts_image, np_pts_ground)
        print("Homography Transformer Initialized")
        return homography


    def image_callback(self, image_msg):
        try:
            # 1. Convert ROS image to OpenCV
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            input_image = image
            height, width = image.shape[:2]

            # 2. Process edges
            cdstP, linesP = self.edges_clean(image)

            # self.get_logger().info(f"linesP {linesP}")
            if linesP is not None:
            #     # 3. Detect closest line and project to ground plane
                closest_line, is_left = self.compute_lane_edges(
                    linesP, width, height)
                if closest_line is not None:
                    cv.line(cdstP, (closest_line[0][0], closest_line[0][1]), (closest_line[1][0], closest_line[1][1]), (255,0,255), 2, cv.LINE_AA)
    
            
                    point1 = self.transform_homography(self.homography, closest_line[0])
                    point2 = self.transform_homography(self.homography, closest_line[1])

            #     # 4. Get lookahead and smooth it
                    lookahead_pt = self.determine_lookahead_point(
                        point1, point2, is_left)
                    weighted_lookahead = self.compute_weighted_lookahead(lookahead_pt)

            #     # 5. Project lookahead point to image space
                    pixel_x, pixel_y = self.transform_homography(self.inv_homography, weighted_lookahead)
                    lookahead_pix = Float32MultiArray()
                    lookahead_pix.data = [pixel_x, pixel_y]
                    self.lookahead_pixel.publish(lookahead_pix)
                    cv.circle(cdstP, (int(round(pixel_x)), int(round(pixel_y))), radius=5, color=(0, 255, 0), thickness=-1)
                    speed = Float32()
                    speed.data = 4.0#2.0#2.5
                    self.speed_pub.publish(speed)
                    lookahead_msg = Float32MultiArray()
                    lookahead_msg.data = weighted_lookahead.tolist()
                    self.lookahead_pub.publish(lookahead_msg)
                # debug_msg = self.bridge.cv2_to_imgmsg(cdstP, "bgr8")
                # self.debug_pub.publish(debug_msg)
        except Exception as e:
            # pass
            self.get_logger().info(f"Error in image_callback {e}")
            return


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
        

        # Convert to HSV
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Zero out the top half of the image (likely not relevant to track lines)
        h, w = hsv.shape[:2]
        hsv[:h//2, :] = 0

        # Define relaxed white HSV range
        lower_white = np.array([0, 0, 170], dtype=np.uint8)
        upper_white = np.array([180, 50, 255], dtype=np.uint8)
        mask_hsv = cv.inRange(hsv, lower_white, upper_white)

        # Optional: Apply a grayscale brightness mask for robustness
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        _, bright_mask = cv.threshold(gray, 200, 255, cv.THRESH_BINARY)
        combined_mask = cv.bitwise_and(mask_hsv, bright_mask)

        # Morphological closing to fill gaps in lines
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        cleaned_mask = cv.morphologyEx(combined_mask, cv.MORPH_CLOSE, kernel)

        # Optional: smooth before edge detection or Hough
        blurred = cv.GaussianBlur(cleaned_mask, (3, 3), 0)

        # Detect lines
        linesP = cv.HoughLinesP(blurred, 1, np.pi / 180, 50, None, 100, 50)

        # Visualize result
        cdstP_vis = cv.cvtColor(cleaned_mask, cv.COLOR_GRAY2BGR)
        if linesP is not None:
            for i in range(len(linesP)):
                l = linesP[i][0]
                cv.line(cdstP_vis, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 2, cv.LINE_AA)

        return cdstP_vis, linesP

    
    def compute_lane_edges(self, lines, image_width, image_height):
        # lines shape: (N, 1, 4), where 4 = (x1, y1, x2, y2)
        lines = np.squeeze(lines)  # (N, 4)
        if lines.ndim == 1:
            lines = lines.reshape(1,4)
        num_lines = len(lines)
        self.get_logger().info(f"Original num lines {num_lines}")
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
        num_lines = len(lines)
        # self.get_logger().info(f"After filtering horizontal num lines {num_lines}")
        dx = dx[non_horizontal_mask]
        dy = dy[non_horizontal_mask]
        x1 = x1[non_horizontal_mask]
        y1 = y1[non_horizontal_mask]
        y2 = y2[non_horizontal_mask]

        non_vertical_mask = (abs(dx) >= 5)

        lines = lines[non_vertical_mask]
        num_lines = len(lines)
        # self.get_logger().info(f"After filtering vertical num lines {num_lines}")
        dx = dx[non_vertical_mask]
        dy = dy[non_vertical_mask]
        x1 = x1[non_vertical_mask]
        y1 = y1[non_vertical_mask]
        y2 = y2[non_vertical_mask]

        slope = np.divide(dy, dx)

        vertical_enough_mask = (abs(slope) > 0.4)

        lines = lines[vertical_enough_mask]
        num_lines = len(lines)
        # self.get_logger().info(f"After filtering too shallow num lines {num_lines}")
        dx = dx[vertical_enough_mask]
        dy = dy[vertical_enough_mask]
        x1 = x1[vertical_enough_mask]
        y1 = y1[vertical_enough_mask]
        y2 = y2[vertical_enough_mask]
        slope = slope[vertical_enough_mask]
        self.get_logger().info(f" vertical enough lines {len(lines)}")


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
        # self.get_logger().info(f"closest line slope {slope[max_index]}")
        return closest_line, is_left  # shape (2, 4)
    
    def transform_homography(self, homography, point):
        u = point[0]
        v = point[1]
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(homography, homogeneous_point)
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
            offset = -0.33
        else:
            offset = 0.37
        lookahead_point = [self.lookahead_dist, y_intercept + offset]
        return lookahead_point

    def compute_weighted_lookahead(self, new_lookahead_point):

        # TODO: if the most recently computed point is very far from the rest, don't use the raw calculation

        if np.linalg.norm(new_lookahead_point - self.lookahead_pt_history[0]) < 0.3:
            newest_point = new_lookahead_point
        else:
            newest_point = self.lookahead_pt_history[0]
        # newest_point = new_lookahead_point

        self.lookahead_pt_history = np.roll(self.lookahead_pt_history, 1, axis=0)
        self.lookahead_pt_history[0] = newest_point
        weights = np.ones(10) * (0.1 / 7)  # Remaining 17 entries share 0.4
        weights[0] = 0.4  # Newest
        weights[1] = 0.25  # 2nd newest
        weights[2] = 0.15  # 3rd newest

        weighted_point = np.sum(self.lookahead_pt_history * weights[:, np.newaxis], axis=0)
        return weighted_point
    
def main(args=None):
    rclpy.init(args=args)
    track_segmentor = TrackSegment()
    rclpy.spin(track_segmentor)
    rclpy.shutdown()