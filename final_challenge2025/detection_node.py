import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from .model.detector import Detector
from std_msgs.msg import String, Bool
import cv2 as cv
import numpy as np

class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        self.banana_publisher = self.create_publisher(Pose, "/detections/banana", 1) #None TODO
        self.traffic_light_publisher = self.create_publisher(Bool, "/detections/traffic_light", 1)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.debug_pub = self.create_publisher(Image, "/debug_img", 10)
        self.bridge = CvBridge()
        self.threshold = 0.1

        self.get_logger().info("Detector Initialized")

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        #Initialize detector
        detector_model = Detector()
        detector_model.set_threshold(self.threshold)

        #Get predictions
        prediction_dict = detector_model.predict(image)
        predictions = prediction_dict["predictions"]

        #Assuming only one banana in frame at a time
        banana_img_pos = None
        banana_found = False
        traffic_light_img_pos = None
        traffic_light_found = False
        
        for detection in predictions:
            if not banana_found and detection[1] == "banana":
                banana_img_pos = detection[0]
            elif detection[1] == "traffic light":
                if traffic_light_found and traffic_light_img_pos[2] < detection[0][2]: #Gets closest traffic light (using x_2)
                    traffic_light_img_pos = detection[0]
                elif not traffic_light_found:
                    traffic_light_found = True
                    traffic_light_img_pos = detection[0]

        #Check detected to publish
        banana_pos_to_pub = Pose()
        banana_pos_to_pub.position.z = 0.0
        banana_pos_to_pub.orientation.x = 1.0
        banana_pos_to_pub.orientation.y = 0.0
        banana_pos_to_pub.orientation.z = 0.0
        banana_pos_to_pub.orientation.w = 0.0
        if banana_img_pos is None:
            banana_pos_to_pub.position.x = 0.0
            banana_pos_to_pub.position.y = 0.0
        else: 
            banana_box_base_x = (banana_img_pos[0] + banana_img_pos[2])/2
            banana_box_base_y = banana_img_pos[3]

            #Highlights banana in image
            cv.rectangle(image, (int(banana_img_pos[0]), int(banana_img_pos[1])), (int(banana_img_pos[2]), int(banana_img_pos[3])), (0,0,255), 5)

            banana_pos_to_pub = Pose()
            banana_pos_to_pub.position.x = float(banana_box_base_x)
            banana_pos_to_pub.position.y = float(banana_box_base_y)

        traffic_state_to_pub = Bool()
        if traffic_light_img_pos is None:
             traffic_state_to_pub.data = False #"Go"
        else:
            # # traffic_state_to_pub.data = traffic_light_checker(image, traffic_light_img_pos)
            # cv.rectangle(image, (int(traffic_light_img_pos[0]),int(traffic_light_img_pos[1])), (int(traffic_light_img_pos[2]), int(traffic_light_img_pos[3])),(0,0,255), 3) #Gets traffic light
            
            # check_red = True
            # red_bb = self.traffic_light_checker(image, traffic_light_img_pos, check_red)
            # if red_bb != ((0,0),(0,0)):
            #     # #Outlines the Traffic light in image
            #     # cv.rectangle(image, red_bb[0], red_bb[1],(0,0,255), 3) #Gets red on traffic light
            #     traffic_state_to_pub.data = True #"Stop"    
            # else:
            #     traffic_state_to_pub.data = False #"Go"

            check_red = False
            green_bb = self.traffic_light_checker(image, traffic_light_img_pos, check_red)
            if green_bb != ((0,0),(0,0)):
                # #Outlines the Traffic light in image
                # cv.rectangle(image, green_bb[0], green_bb[1],(0,0,255), 3) #Gets red on traffic light
                traffic_state_to_pub.data = False#"Go"    
            else:
                traffic_state_to_pub.data = True #"Stop"

        #publish data
        self.banana_publisher.publish(banana_pos_to_pub)
        self.traffic_light_publisher.publish(traffic_state_to_pub)
        debug_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
        self.debug_pub.publish(debug_msg)


    def traffic_light_checker(self,image, image_position, check_red):
        """
        image: np list
        image_position: (x1,y1,x2,y2)
        Will act as a color segmentor
        """
        traffic_light_img = image[int(image_position[0]):int(image_position[2]), 
                                int(image_position[1]): int(image_position[3])]
        
        if check_red:
            red_HSV_lower = np.array([0,0,200]) 
            red_HSV_upper = np.array([10,255,255])
            bb = fc_TL_color_segmentation(traffic_light_img, (red_HSV_lower, red_HSV_upper))
        else:
            green_HSV_lower = np.array([75,0,200]) 
            green_HSV_upper = np.array([90,255,255])
            bb = fc_TL_color_segmentation(traffic_light_img, (green_HSV_lower, green_HSV_upper))

        return bb

    


# def image_print(img):
# 	"""
# 	Helper function to print out images, for debugging. Pass them in as a list.
# 	Press any key to continue.
# 	"""
# 	cv.imshow("image", img)
# 	cv.waitKey(0)
# 	cv.destroyAllWindows()

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
	kernel = cv.getStructuringElement(cv.MORPH_RECT,(5,5))
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