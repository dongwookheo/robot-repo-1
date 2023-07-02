import socket
import struct
import numpy as np
import cv2

# Set up server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.0.45', 8485))  # 서버의 IP주소와 포트번호 입력
server_socket.listen(1)  # listen() 인자는 서버가 대기할 최대 연결 수

# Accept client connection
print("Waiting for a connection...")
connection, client_address = server_socket.accept()
print("Client connection accepted:", client_address)

try:
    while True:
        # Receive and unpack color data
        data = connection.recv(8)
        if not data:
            break
        color_image_size = struct.unpack('II', data)
        color_image_bytes = connection.recv(color_image_size[0] * color_image_size[1] * 3)
        color_image = np.frombuffer(color_image_bytes, dtype=np.uint8)
        color_image = np.reshape(color_image, (color_image_size[0], color_image_size[1], 3))

        # Receive and unpack depth data
        data = connection.recv(8)
        if not data:
            break
        depth_image_size = struct.unpack('II', data)
        depth_image_bytes = connection.recv(depth_image_size[0] * depth_image_size[1] * 3)
        depth_image = np.frombuffer(depth_image_bytes, dtype=np.uint8)
        depth_image = np.reshape(depth_image, (depth_image_size[0], depth_image_size[1], 3))

        # Display the received images
        cv2.imshow('Received Color Image', color_image)
        cv2.imshow('Received Depth Image', depth_image)
        cv2.waitKey(1)

finally:
    # Clean up the connection and server sockets
    connection.close()
    server_socket.close()
    cv2.destroyAllWindows()
