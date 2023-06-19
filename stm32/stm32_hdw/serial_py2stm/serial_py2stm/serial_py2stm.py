import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import numpy as np
import math

ser = None

class CmdVelSerialSender(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_sender')
        self.subscription = self.create_subscription(
            Twist,
            # '/turtle1/cmd_vel',       # test with turtlesim
            'cmd_vel',                  # teleop_twist_keyboard
            self.cmd_vel_callback,
            10)
        self.subscription   # prevent unused variable warning
        

    def cmd_vel_callback(self, twist):
        wheel_radius = 0.04
        wheel_separation_width = 0.095      # distance left to right wheel / 2
        wheel_separation_length = 0.0875    # distance front to rear wheel / 2
        fl_wheel_vel = (1 / wheel_radius) * (twist.linear.x - twist.linear.y - 
                                            (wheel_separation_width + wheel_separation_length) * twist.angular.z)
        fr_wheel_vel = (1 / wheel_radius) * (twist.linear.x + twist.linear.y + 
                                            (wheel_separation_width + wheel_separation_length) * twist.angular.z)
        rl_wheel_vel = (1 / wheel_radius) * (twist.linear.x + twist.linear.y -
                                            (wheel_separation_width + wheel_separation_length) * twist.angular.z)
        rr_wheel_vel = (1 / wheel_radius) * (twist.linear.x - twist.linear.y +
                                            (wheel_separation_width + wheel_separation_length) * twist.angular.z)
        
        # rad/s -> rpm transfrom & 0~333rpm -> 0~1000pwm mapping
        fl = np.int16(fl_wheel_vel * 60 * 1000 / (2 * math.pi * 333))
        fr = np.int16(fr_wheel_vel * 60 * 1000 / (2 * math.pi * 333) * (-1))
        rl = np.int16(rl_wheel_vel * 60 * 1000 / (2 * math.pi * 333))
        rr = np.int16(rr_wheel_vel * 60 * 1000 / (2 * math.pi * 333) * (-1))
    
        # Convert Twist messages to strings
        command = [0xfa, 0xfe, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0xfa, 0xfd]

        command[3] = (np.uint8)(fl >> 8)
        command[4] = (np.uint8)(fl)
        command[5] = (np.uint8)(fr >> 8)
        command[6] = (np.uint8)(fr)
        command[7] = (np.uint8)(rl >> 8)
        command[8] = (np.uint8)(rl)
        command[9] = (np.uint8)(rr >> 8)
        command[10] = (np.uint8)(rr)
        command[13] = (np.uint8)(sum(command[2:13]))
        
        # Send data to STM32f103rct6 using serial communication
        print(command)
        global ser
        ser.write(bytes(command))

def main(args=None):
    # Initialize ROS nodes and serial ports
    rp.init(args=args)
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 1000000)
    
    py2stm_subscriber = CmdVelSerialSender()
    rp.spin(py2stm_subscriber)
    py2stm_subscriber.destroy_node()
    rp.shutdown()
    
if __name__ == '__main__':
    main()