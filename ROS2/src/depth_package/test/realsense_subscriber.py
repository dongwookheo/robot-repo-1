import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2

from cv_bridge import CvBridge, CvBridgeError

class RealsenseSubscriber(Node):

    def __init__(self):
        super().__init__('realsense_subscriber')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/image/color", self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/image/depth", self.depth_callback, 10)
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv2.imshow("Color Image", cv_image)
        self.key = cv2.waitKey(1) & 0xFF
        
    
    def depth_callback(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv2.imshow("Depth Image", cv_depth)
        self.key = cv2.waitKey(1) & 0xFF

def main(args=None):
    rp.init(args=args)
    realsense_subscriber = RealsenseSubscriber()
    while(1):
        rp.spin_once(realsense_subscriber)
        if realsense_subscriber.key == ord('q'):
            break

    realsense_subscriber.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()