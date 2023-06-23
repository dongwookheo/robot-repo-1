import rclpy as rp
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealsensePublisher(Node):
    
    def __init__(self):
        super().__init__('realsense_publisher')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/image", 10)
        self.depth_pub = self.create_publisher(Image, "/depth", 10)
        self.pipeline = rs.pipeline()

    def start_camera(self):
        config = rs.config()
        config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)

        self.pipeline.start(config)

        try:
            while rp.ok():

                align_to = rs.stream.color
                align = rs.align(align_to)
            
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                image_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
                depth_msg = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")

                self.image_pub.publish(image_msg)
                self.depth_pub.publish(depth_msg)

                # test imshow
                # cv2.imshow('color', color_image); cv2.imshow('depth', depth_colormap)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
        finally:
            self.pipeline.stop()


def main(args=None):
    rp.init(args=args)
    realsense_camera = RealsensePublisher()
    realsense_camera.start_camera()

    realsense_camera.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()
