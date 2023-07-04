import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time
from ultralytics import YOLO
from cv_bridge import CvBridge

# Load the YOLOv8 model
model = YOLO('/home/hdw/lidar_study_ws/src/yolov8m.pt')
start_time = time.time()
total_frames = 0

bridge = CvBridge()

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, data):
        global total_frames, start_time
        image = bridge.imgmsg_to_cv2(data, 'bgr8')
        resized_image = cv2.resize(image, (320, 240))
        
        # Run YOLOv8 inference on the frame
        results = model(resized_image)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        
        total_frames += 1

        term = time.time() - start_time
        fps = total_frames / term
        fps_string = f'term = {term:.3f},  FPS = {fps:.2f}'
        print(fps_string)

        cv2.putText(annotated_frame, fps_string, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255))

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        cv2.waitKey(33)

def main(args=None):
    rp.init(args=args)
    node = ImageSubscriber()

    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by Keyboard')
    finally:
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
