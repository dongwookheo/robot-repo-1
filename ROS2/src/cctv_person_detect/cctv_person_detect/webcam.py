import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.webcam = cv2.VideoCapture(2)

    def timer_callback(self):
        ret, frame = self.webcam.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from webcam')
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: {0}".format(e))
            return

        self.publisher.publish(img_msg)

    def __del__(self):
        self.webcam.release()

def main(args=None):
    rp.init(args=args)
    webcam_publisher = WebcamPublisher()
    rp.spin(webcam_publisher)

    webcam_publisher.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
