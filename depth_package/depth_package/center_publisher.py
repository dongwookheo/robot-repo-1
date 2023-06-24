# Run on PC

import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
from realsense_person_detect_msgs.msg import CenterCoordinate, CoordinateList
import numpy as np
import datetime
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError

class CenterPublisher(Node):

    def __init__(self):
        super().__init__('depth_publisher')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/image/color", self.color_image_callback, 10)
        # self.depth_sub = self.create_subscription(Image, "/image/depth", self.depth_image_callback, 10)
        self.center_pub = self.create_publisher(CoordinateList,
                                                "/center_coord_list",
                                                 10)
        self.count = 1
        self.center_coord = CenterCoordinate()
        self.center_coord_list = CoordinateList()

        # Load the YOLOv8 model
        self.model = YOLO('/home/hdw/Downloads/bestx_v3.pt')
        
    def color_image_callback(self, msg):
        start_time = datetime.datetime.now()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Get detected objects
            detections = self.model(cv_image)[0]
            # annotated_frame = detections.plot()

            # List to store the coordinate of arms & legs
            point_list = []

            CONFIDENCE_THRESHOLD = 0.5
            RED = (0, 0, 255); GREEN = (0, 255, 0); BLUE = (255, 0, 0)
            for data in detections.boxes.data.tolist():
                confidence = float(data[4])
                if confidence < CONFIDENCE_THRESHOLD:
                    continue
                label = int(data[5])
                if label != 0 and label != 1:
                # if label != 0:      # cocodataset
                    continue

                xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
                cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), GREEN, 2)
                if label == 0:
                    cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), GREEN, 2)
                    cv2.putText(cv_image, 'arms'+ ' ' + str(round(confidence, 2))+'%', (xmin, ymin), cv2.FONT_HERSHEY_PLAIN, 1.5, BLUE, 2)
                else:
                    cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), RED, 2)
                    cv2.putText(cv_image, 'legs'+ ' ' + str(round(confidence, 2))+'%', (xmin, ymin), cv2.FONT_HERSHEY_PLAIN, 1.5, BLUE, 2)
                # center coordinate 
                center_x = (xmin + xmax) // 2
                center_y = (ymin + ymax) // 2
                point_list.append((center_x, center_y))
                
                self.assign_coord(point_list)
    
        except CvBridgeError as e:
            print(e)
            return
        
        finally:
            self.center_pub.publish(self.center_coord_list)

            # Calculate fps
            end_time = datetime.datetime.now()
            total = (end_time - start_time).total_seconds()
            fps = f'FPS: {1 / total:.2f}'
            cv2.putText(cv_image, fps, (10, 10), cv2.FONT_HERSHEY_PLAIN, 1, BLUE, 2)

            cv2.imshow("Recieved Color Image", cv_image)
            self.key = cv2.waitKey(1) & 0xFF
        
    
    def depth_image_callback(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv2.imshow("Depth Image", cv_depth)
        self.key = cv2.waitKey(1) & 0xFF

    def assign_coord(self, coord_list):
        self.center_coord_list = CoordinateList()
        self.count += 1
        for point in coord_list:
            center_coord = CenterCoordinate()
            x, y = point
            center_coord.x = int(x)
            center_coord.y = int(y)

            self.center_coord_list.msg_seq = self.count
            self.center_coord_list.coord_list.append(center_coord)

def main(args=None):
    rp.init(args=args)
    center_publisher = CenterPublisher()
    while(1):
        rp.spin_once(center_publisher)
        if center_publisher.key == ord('q'):
            break

    center_publisher.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()