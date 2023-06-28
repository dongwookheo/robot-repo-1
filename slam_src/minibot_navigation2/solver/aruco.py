import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('arucodetector')
        self.subscription = self.create_subscription(
            Image,
            '/image/color',
            self.image_callback,
            10
        )
        #self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Int32, 'aruco_id', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
            parameters = cv2.aruco.DetectorParameters()

            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Publish marker IDs
                if len(ids) > 0 and len(ids[0]) > 0:
                    marker_id = ids[0][0]
                    print("Detected marker ID:", marker_id)

                    marker_id_msg = Int32()
                    marker_id_msg.data = int(ids[0][0])
                    self.publisher.publish(marker_id_msg)

            cv2.imshow('ArUco Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error('Error processing image: {0}'.format(e))

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()