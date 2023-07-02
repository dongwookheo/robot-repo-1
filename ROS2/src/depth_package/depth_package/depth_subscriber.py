import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import UInt16

class DepthSubscriber(Node):

    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            UInt16,
            '/depth',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        depth = msg.data
        print(f'Received depth: {depth}') # 출력

def main(args=None):
    rp.init(args=args)
    depth_subscriber = DepthSubscriber()

    rp.spin(depth_subscriber)

    depth_subscriber.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()
