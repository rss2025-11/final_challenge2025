import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from .detector import Detector
from std_msgs.msg import String, Bool, Float32MultiArray
import cv2 as cv
import numpy as np
import os
import time

class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        # self.banana_publisher = self.create_publisher(Pose, "/detections/banana", 1) #None TODO
        self.banana_publisher = self.create_publisher(Float32MultiArray, "/detections/banana", 1) #None TODO

        self.traffic_light_publisher = self.create_publisher(Bool, "/detections/traffic_light", 1)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.subscriber = self.create_subscription(Bool, "/banana_save_img", self.recieve_save_rqst, 1)
        # self.debug_pub = self.create_publisher(Image, "/debug_img", 10)
        # self.tl_debug_pub = self.create_publisher(Image, "/tl_debug_img", 10)
        self.off_detection_sub = self.create_subscription(Bool, "/process_image", self.process_image_callback, 1)
        self.bridge = CvBridge()
        self.threshold = 0.3
        self.homography = self.create_homography_matrix()
        self.banana_counter = 0
        # self.tl_counter = 0
        # self.last_time_dtct = 0
        self.save_rqsts = 0
        self.on_processing = True

        #Initialize detector
        self.detector_model = Detector(from_tensor_rt=False) # Detector()
        self.detector_model.to('cuda')
        self.detector_model.set_threshold(self.threshold)

    def process_image_callback(self, msg):
        self.on_processing = msg.data
        if self.on_processing:
            self.get_logger().info("Detector Initialized")
    def callback(self, img_msg):
        if not self.on_processing:
            return
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        #Get predictions
        prediction_dict = self.detector_model.predict(image)
        predictions = prediction_dict["predictions"]

        #Assuming only one banana in frame at a time
        banana_img_pos = None
        banana_found = False
        traffic_light_img_pos = None
        traffic_light_found = False
        
        for detection in predictions:
            if not banana_found and detection[1] == "banana":
                banana_img_pos = detection[0]
                banana_img_conf = detection[2]
            elif detection[1] == "traffic light":
                if traffic_light_found and traffic_light_img_pos[2] < detection[0][2]: #Gets closest traffic light (using x_2)
                    traffic_light_img_pos = detection[0]
                elif not traffic_light_found:
                    traffic_light_found = True
                    traffic_light_img_pos = detection[0]
                self.get_logger().info(f'Traffic light conf: {detection[2]}')

        #Check detected to publish
        banana_pos_to_pub = Pose()
        banana_pos_to_pub.position.z = 0.0
        banana_pos_to_pub.orientation.x = 1.0
        banana_pos_to_pub.orientation.y = 0.0
        banana_pos_to_pub.orientation.z = 0.0
        banana_pos_to_pub.orientation.w = 0.0
        relative_x = 0.0
        relative_y = 0.0
        if banana_img_pos is None:
            banana_pos_to_pub.position.x = 0.0
            banana_pos_to_pub.position.y = 0.0
        else: 
            banana_box_base_x = (banana_img_pos[0] + banana_img_pos[2])/2
            banana_box_base_y = banana_img_pos[3]

            #Highlights banana in image
            cv.rectangle(image, (int(banana_img_pos[0]), int(banana_img_pos[1])), (int(banana_img_pos[2]), int(banana_img_pos[3])), (0,0,255), 5)

            if(self.save_rqsts > 0):
                self.save_rqsts -= 1
                self.save_img(image)

            # Converting pixel coordinates to real world coordinates
            point = np.array([banana_box_base_x, banana_box_base_y])
            relative_x, relative_y = self.transform_homography(self.homography, point)

            banana_pos_to_pub = Pose()
            banana_pos_to_pub.position.x = relative_x #float(banana_box_base_x)
            banana_pos_to_pub.position.y = relative_y #float(banana_box_base_y)
            banana_pos_to_pub.position.z = float(banana_img_conf)

            banana_msg = Float32MultiArray()
            banana_msg.data = [relative_x, relative_y]
            self.banana_publisher.publish(banana_msg)


        traffic_state_to_pub = Bool()
        if traffic_light_img_pos is None:
            traffic_state_to_pub.data = False #"Go"

            self.traffic_light_publisher.publish(traffic_state_to_pub)
        else:
            # traffic_state_to_pub.data = traffic_light_checker(image, traffic_light_img_pos)
            # cv.rectangle(image, (int(traffic_light_img_pos[0]),int(traffic_light_img_pos[1])), (int(traffic_light_img_pos[2]), int(traffic_light_img_pos[3])),(0,0,255), 3) #Gets traffic light
            
            check_red = True
            red_bb = self.traffic_light_checker(image, traffic_light_img_pos, check_red)
            if red_bb != ((0,0),(0,0)) and abs(red_bb[0][0] - red_bb[1][0]) >= 5:
                # #Outlines the Traffic light in image
                # cv.rectangle(image, red_bb[0], red_bb[1],(0,0,255), 3) #Gets red on traffic light
                traffic_state_to_pub.data = True #"Stop"   
            else:
                traffic_state_to_pub.data = False #"Go"

            self.traffic_light_publisher.publish(traffic_state_to_pub)
            # debug_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
            # self.debug_pub.publish(debug_msg) 
            

    # TODO: Move homography matrix and transformaiton to its own separate file/utils for organization
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
        self.get_logger().info(f'')
        return homography

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


    def traffic_light_checker(self,image, image_position, check_red):
        """
        image: np list
        image_position: (x1,y1,x2,y2)
        Will act as a color segmentor
        """
        # Want to get the traffic light, Assuming that's where red is
        traffic_light_img = image[int(image_position[1]):int(image_position[3]), 
                                int(image_position[0]): int(image_position[2])]
        
        if check_red:
            red_HSV_lower = np.array([100,0,217]) #(105,0,217)
            red_HSV_upper = np.array([130,256,256]) #(115,256,256)
            bb = fc_TL_color_segmentation(traffic_light_img, (red_HSV_lower, red_HSV_upper))
        else:
            green_HSV_lower = np.array([20,0,230]) 
            green_HSV_upper = np.array([40,256,256])
            bb = fc_TL_color_segmentation(traffic_light_img, (green_HSV_lower, green_HSV_upper))

        return bb

    def recieve_save_rqst(self, msg):
        if(msg.data):
            self.save_rqsts += 1

    def save_img(self, img, banana=True):
        cur_time = time.time()
        if banana:
            save_path = f"{os.path.dirname(__file__)}/bananarun_{self.banana_counter}.png"
            # img.save(save_path)
            # self.get_logger().info(f"Saved banana {self.banana_counter} to {save_path}!")
            rgb_img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
            cv.imwrite(save_path, rgb_img)
            self.banana_counter += 1
        elif  cur_time - self.last_time_dtct > 1:
            save_path = f"{os.path.dirname(__file__)}/TL_{self.tl_counter}.png"
            hsv_img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
            cv.imwrite(save_path, hsv_img)
            self.tl_counter += 1
            self.last_time_dtct = cur_time

def fc_TL_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	#initalize
	min_x, max_x = 0,0
	min_y, max_y = 0,0
	hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

	#Find orange pixels
	orange_mask = cv.inRange(hsv_img, template[0], template[1])
     
	#Open img (Erode then dilate)
	kernel = cv.getStructuringElement(cv.MORPH_RECT,(1,1))
	processed_mask = cv.morphologyEx(orange_mask, cv.MORPH_OPEN, kernel)
	processed_mask = cv.morphologyEx(processed_mask, cv.MORPH_CLOSE, kernel)


	#Counture and find bounding rect
	contours, _ = cv.findContours(processed_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)


	if(len(contours) > 0):
		# cnt = contours[0]
		cnt = max(contours, key=cv.contourArea)

		x,y,w,h = cv.boundingRect(cnt)
		bounding_box = ((x,y),(x + w, y + h))

	else: 
		bounding_box = ((0,0),(0,0))
	########## YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box


def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()