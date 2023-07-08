import rclpy
from rclpy.node import Node
import tf2_ros
from math import pi
from tf2_ros import TransformException
from tf2_geometry_msgs import TransformStamped, do_transform_pose

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


waypoints = [  # This is our list of waypoints
    [(0.4, 0.4, 0.0), tf2_ros.TransformStamped(), 'kiri dalam'],
    [(2.5, 0.4, 0.0), tf2_ros.TransformStamped(), 'kiri atas luar'],
    [(0.4, 0.4, 0.0), tf2_ros.TransformStamped(), 'kiri dalam'],
    [(0.4, 2.5, 0.0), tf2_ros.TransformStamped(), 'kiri kiri luar'],
    [(0.4, -0.4, 0.0), tf2_ros.TransformStamped(), 'kanan dalam'],
    [(2.5, -0.4, 0.0), tf2_ros.TransformStamped(), 'kanan atas luar'],
    [(0.4, -0.4, 0.0), tf2_ros.TransformStamped(), 'kanan dalam'],
    [(0.4, -2.5, 0.0), tf2_ros.TransformStamped(), 'kanan kanan luar'],
]


def goal_pose(pose):  # Convert from waypoints into goal positions
    goal_pose = NavigateToPose.Goal()
    goal_pose.pose.header.frame_id = 'map'
    goal_pose.pose.pose.position.x = pose[0][0]
    goal_pose.pose.pose.position.y = pose[0][1]
    goal_pose.pose.pose.position.z = pose[0][2]
    goal_pose.pose.pose.orientation.x = pose[1].transform.rotation.x
    goal_pose.pose.pose.orientation.y = pose[1].transform.rotation.y
    goal_pose.pose.pose.orientation.z = pose[1].transform.rotation.z
    goal_pose.pose.pose.orientation.w = pose[1].transform.rotation.w

    return goal_pose


class MazePatrol(Node):
    def __init__(self):
        super().__init__('maze_patrol')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Navigate To Pose action server not available, waiting...')
        self.send_goals()

    def send_goals(self):
        for pose in waypoints:
            try:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'base_link'
                pose_stamped.pose.position.x = pose[0][0]
                pose_stamped.pose.position.y = pose[0][1]
                pose_stamped.pose.position.z = pose[0][2]
                pose[1] = self.tf_buffer.transform(pose_stamped, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
            except (TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as ex:
                self.get_logger().error(f"Failed to transform pose: {ex}")
                continue

            goal = goal_pose(pose)
            self.get_logger().info(f"Moving towards goal: {pose[2]}")
            self.get_logger().info('Waiting for result...')
            self.client.wait_for_result()

        self.get_logger().info("Completed.")


def main(args=None):
    rclpy.init(args=args)
    maze_patrol = MazePatrol()
    rclpy.spin(maze_patrol)
    maze_patrol.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()