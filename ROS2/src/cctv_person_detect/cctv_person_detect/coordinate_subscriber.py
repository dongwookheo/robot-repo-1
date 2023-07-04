import rclpy as rp
from rclpy.node import Node

# custom person coordinate msg
from cctv_person_detect_msgs.msg import PersonCoordinate, CoordinateList

class CoordinateSubscriber(Node):
    def __init__(self):
        super().__init__('coordinate_subscriber')
        self.subscription = self.create_subscription(
            CoordinateList,
            '/person_coord_list',
            self.person_coord_callback,
            10)
        
        self.subscription   # prevent unused variable warning

    def person_coord_callback(self, msg):
        coord_list_size = len(msg.coord_list)
        print("-"*10)
        print("Message Sequence: ", msg.msg_seq)
        print("Person Coordinates:")
        for i in range(coord_list_size):
            print("(", msg.coord_list[i].person_x, ", ", 
                  msg.coord_list[i].person_y, ")", end=", ")
        print("\n")

def main(args=None):
    rp.init(args=args)
    coordinate_subscriber = CoordinateSubscriber()
    rp.spin(coordinate_subscriber)

    coordinate_subscriber.destroy_node()
    rp.shutdown()

if __name__ == "__main__":
    main()
