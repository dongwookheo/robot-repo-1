import socket
import struct
import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Setup client socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ip= '192.168.0.45'; port = 8485
server_address = (server_ip, port)
client_socket.connect(server_address)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        
        try:
            cv2.imshow('color', resized_color_image); cv2.imshow('depth', depth_image)
        except:
            cv2.imshow('color', color_image); cv2.imshow('depth', depth_colormap)
        
        # Package color and depth data for transmission
        data = struct.pack('II', color_image.shape[0], color_image.shape[1]) + color_image.tobytes()
        data += struct.pack('II', depth_colormap.shape[0], depth_colormap.shape[1]) + depth_colormap.tobytes()
        
        # Send the packaged data to server
        client_socket.sendall(data)
        
        key = cv2.waitKey(1) & 0xFF 
        if key == ord('q'):
            break

finally:

    # Stop streaming
    pipeline.stop()
    
    # Close the socket connection
    client_socket.close()
    
    cv2.destroyAllWindows()