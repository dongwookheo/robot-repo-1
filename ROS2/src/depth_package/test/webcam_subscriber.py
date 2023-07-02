import rclpy as rp
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class WebcamSubscriber(Node):

    def __init__(self, topic_name, display_name):
        super().__init__('webcam_subscriber_' + display_name)
        self.subscription = self.create_subscription(
            Image, topic_name, self.image_callback, 10)
        self.subscription
        self.bridge = CvBridge()
        self.display_name = display_name

    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv2.imshow(self.display_name, cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error("CvBridgeError: %s" % e)


def main(args=None):
    rp.init(args=args)

    webcam_subscriber_0 = WebcamSubscriber('webcam/frame_0', 'Webcam_0')
    webcam_subscriber_2 = WebcamSubscriber('webcam/frame_2', 'Webcam_2')

    while(1):
        rp.spin_once(webcam_subscriber_0)
        rp.spin_once(webcam_subscriber_2)

    # Destroy the nodes explicitly
    webcam_subscriber_0.destroy_node()
    webcam_subscriber_2.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()