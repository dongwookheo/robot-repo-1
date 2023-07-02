import rclpy as rp
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from ultralytics import YOLO

# from std_msgs.msg import Int16


class WebcamPublisher(Node):
    def __init__(self, webcam_idx, topic_name):
        super().__init__('webcam_publisher_' + str(webcam_idx))
        self.publisher_ = self.create_publisher(Image, 
                                                topic_name, 
                                                10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        self.cap = cv2.VideoCapture(webcam_idx)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(image_msg)
            except CvBridgeError as e:
                self.get_logger().error("CvBridgeError: %s" % e)
        else:
            self.get_logger().error("Error capturing frame")

def main(args=None):
    rp.init(args=args)

    webcam_publisher_0 = WebcamPublisher(0, 'webcam/frame_0')
    webcam_publisher_2 = WebcamPublisher(2, 'webcam/frame_2')

    while(1):
        rp.spin_once(webcam_publisher_0)
        rp.spin_once(webcam_publisher_2)
    
    # Destroy the nodes
    webcam_publisher_0.destroy_node()
    webcam_publisher_2.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()