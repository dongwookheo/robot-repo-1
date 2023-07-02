# Run on Jetson

import rclpy as rp
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt16
from realsense_person_detect_msgs.msg import CenterCoordinate, CoordinateList

class RealsensePublisher(Node):
    
    def __init__(self):
        super().__init__('realsense_publisher')
        self.bridge = CvBridge()
        self.color_image_pub = self.create_publisher(Image, "/image/color", 10)
        self.depth_image_pub = self.create_publisher(Image, "/image/depth", 10)
        self.center_sub = self.create_subscription(
            CoordinateList,
            '/center_coord_list',
            self.get_depth,
            10)
        self.depth_pub = self.create_publisher(UInt16, '/depth', 10)
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)

        self.pipeline.start(config)
        self.center_sub     # prevent unused variable warning
        
        # Timer callback
        self.timer = self.create_timer(0.03, self.start_camera)

        
    def start_camera(self):
        align_to = rs.stream.color
        align = rs.align(align_to)
    
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        self.aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        # Validate that both frames are valid
        if self.aligned_depth_frame and color_frame:
            depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            image_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")

            self.color_image_pub.publish(image_msg)
            self.depth_image_pub.publish(depth_msg)

            # test imshow
            # cv2.imshow('color', color_image); cv2.imshow('depth', depth_colormap)
            
        
    def get_depth(self, msg):
        coord_list_size = len(msg.coord_list)
        depth_list = []
        depth = 10000.0
        if self.aligned_depth_frame is not None and coord_list_size > 0:
            # depth_width, depth_height = self.aligned_depth_frame.get_width(), self.aligned_depth_frame.get_height()
            
            for i in range(coord_list_size):
                center_x = msg.coord_list[i].x
                center_y = msg.coord_list[i].y

                # if (center_x >= 0 and center_x < depth_width) and (center_y >= 0 and center_y < depth_height):
                tmp_depth = self.aligned_depth_frame.get_distance(center_x, center_y)
                depth_list.append(tmp_depth)
                depth = tmp_depth if tmp_depth < depth else depth
            print(depth_list)
            print('\n')
            print(depth)
            print('\n')
            depth_uint16 = UInt16()
            depth_uint16.data = int(depth*100)
            self.depth_pub.publish(depth_uint16)
            print(depth_uint16)
            print('-'*10)
        else:
            print("Depth not available.")
                


def main(args=None):
    rp.init(args=args)
    realsense_camera = RealsensePublisher()
    rp.spin(realsense_camera)

    realsense_camera.pipeline.stop()
    realsense_camera.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()
