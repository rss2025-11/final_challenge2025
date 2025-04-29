import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRecorder(Node):
    def __init__(self):
        super().__init__('image_recorder')
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback,
            10)

        self.bridge = CvBridge()
        self.writer = None

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            height, width = cv_image.shape[:2]
            self.writer = cv2.VideoWriter('output.mp4', fourcc, 30, (width, height))

        self.writer.write(cv_image)

    def destroy_node(self):
        if self.writer:
            self.writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = ImageRecorder()
    rclpy.spin(recorder)
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
