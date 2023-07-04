import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import datetime
from ultralytics import YOLO
from cv_bridge import CvBridge

# custom person coordinate msg
from cctv_person_detect_msgs.msg import PersonCoordinate, CoordinateList

# Load previously saved data (internal parameter matrix and distortion coefficients)
cali_mtx = np.load('/home/hdw/lidar_study_ws/src/calibration_matrix.npy')
dist_coef = np.load('/home/hdw/lidar_study_ws/src/distortion_coefficients.npy')

# Define coordinate system - Homography transform
IMAGE_POINTS = [(159, 269),(198, 100),(270, 37),(365, 35),(462, 273)]
# map image (for test)
# TRANSFORMED_POINTS = [(981, 800),(456, 800),(123, 504),(124, 282),(971, 161)]
# real map (on rviz)
TRANSFORMED_POINTS = [(-0.3518632650375366, 1.006995439529419),
                        (-0.3061986267566681, -0.39697912335395813), 
                        (0.5257650017738342, -1.3523194789886475), 
                        (1.162017822265625, -1.3341068029403687), 
                        (1.4726362228393555, 0.9961833357810974)]
H, _ = cv2.findHomography(
    np.array(IMAGE_POINTS), np.array(TRANSFORMED_POINTS), cv2.RANSAC
    )

class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('person_coordinate')
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.timer_period = 1.0
        self.publisher = self.create_publisher(
            PersonCoordinate,
            '/person_coord',
            10
        )
        self.publisher2 = self.create_publisher(
            CoordinateList,
            '/person_coord_list',
            10
        )
        self.count = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Load the YOLOv8 model
        self.model = YOLO('/home/hdw/lidar_study_ws/src/yolov8m.pt')
        self.bridge = CvBridge()

    def image_callback(self, data):
        start_time = datetime.datetime.now()
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        resized_image = cv2.resize(image, (640, 480))

        # Calibration
        global cali_mtx, dist_coef, H
        resized_image = cv2.undistort(resized_image, cali_mtx, dist_coef)

        # Get detected objects
        detections = self.model(resized_image)[0]
        
        # List to store the coordinates of people
        point_list = []
        point_list2 = []        # avoid numpy error
        transformed_point_list = []

        CONFIDENCE_THRESHOLD = 0.5
        GREEN = (0, 255, 0); BLUE = (255, 0, 0)
        for data in detections.boxes.data.tolist():
            confidence = float(data[4])
            if confidence < CONFIDENCE_THRESHOLD:
                continue
            label = int(data[5])
            if label != 0:
                continue

            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
            cv2.rectangle(resized_image, (xmin, ymin), (xmax, ymax), GREEN, 2)
            cv2.putText(resized_image, 'person'+ ' ' + str(round(confidence, 2))+'%', (xmin, ymin), cv2.FONT_HERSHEY_PLAIN, 1.5, BLUE, 2)
            
            # Display bottom center coordinates 
            center_x = (xmin + xmax) // 2
            center_y = ymax
            point_list.append((center_x, center_y))
            point_list2 = np.array(point_list, dtype=np.int32)
            transformed_point_list = cv2.perspectiveTransform(point_list2.reshape(-1, 1, 2).astype(np.float32), H)
            transformed_point_list = transformed_point_list.reshape(-1, 2)
            self.assign_coord(transformed_point_list)

            try:
                for point in transformed_point_list:
                    x, y = point

                    # Draw a transformed point
                    # cv2.circle(image, (int(x), int(y)), radius=10, color=(0, 255, 0), thickness=-1)

                cv2.circle(resized_image, (center_x, center_y), 2, BLUE, -1)
            except:
                pass

        
        # Calculate fps
        end_time = datetime.datetime.now()
        total = (end_time - start_time).total_seconds()
        fps = f'FPS: {1 / total:.2f}'
        cv2.putText(resized_image, fps, (10, 15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

        # Display the annotated frame
        cv2.imshow("YOLOv8", resized_image)
        cv2.waitKey(1)

    def assign_coord(self, coord_list):
        self.person_coord = PersonCoordinate()
        self.person_coord_list = CoordinateList()
        for point in coord_list:
            x, y = point
            self.person_coord.person_x = float(x)
            self.person_coord.person_y = float(y)
            self.person_coord.person_z = 0.0

            self.person_coord_list.msg_seq = self.count
            self.person_coord_list.coord_list.append(self.person_coord)

    def timer_callback(self):
        
        try:
            self.publisher2.publish(self.person_coord_list)
            self.count += 1
        except:
            pass

def main(args=None):
    rp.init(args=args)
    node = CoordinatePublisher()

    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by Keyboard')
    finally:
        node.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()
