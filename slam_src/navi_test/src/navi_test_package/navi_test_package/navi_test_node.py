import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class navi_subscriber(Node) :
    def __init__(self) :
        super().__init__('_subscriber')
        self.subscription = self.create_subscription(
            String, 
            'table_topic',
            self.callback,
            10
        )

        self.goals = []
        self.navigator = BasicNavigator()
        self.setInitialPose(self.navigator)

    def callback(self, msg) :
        self.navigator.waitUntilNav2Active()
        if msg.data == "a table" :
            self.setNavigationGoals_1(self.navigator, self.goals) # 1번 경로 설정

        i = 0
        for goal in self.goals:
            i += 1
            print([goal])
            self.navigator.goThroughPoses([goal])  # Move to the current goal

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                    self.navigator.cancelTask()
                if feedback.distance_remaining < 0.2 or Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                    break

            print('Moving towards goal pose {}'.format(i))
            print('g{}'.format(i))

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def setInitialPose(self, navigator):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -1.6781088109157032
        initial_pose.pose.position.y = 0.585551510681771
        initial_pose.pose.orientation.z = -0.2792168827477615
        initial_pose.pose.orientation.w = 0.9602280626958487
        navigator.setInitialPose(initial_pose)

    def setNavigationGoals_0(self, navigator, goals):
        goal_pose_0 = PoseStamped()
        goal_pose_0.header.frame_id = 'map'
        goal_pose_0.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_0.pose.position.x = -1.6781088109157032
        goal_pose_0.pose.position.y = 0.585551510681771
        goal_pose_0.pose.orientation.z = -0.2792168827477615
        goal_pose_0.pose.orientation.w = 0.9602280626958487
        goals.append(goal_pose_0)

    def setNavigationGoals_1(self, navigator, goals):
        goal_pose_1 = PoseStamped()
        goal_pose_1.header.frame_id = 'map'
        goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_1.pose.position.x = -0.9990098476409912
        goal_pose_1.pose.position.y = 0.057299137115478516
        goal_pose_1.pose.orientation.z = -0.28940227799286045
        goal_pose_1.pose.orientation.w = 0.9572075644772888
        goals.append(goal_pose_1)

        goal_pose_2 = PoseStamped()
        goal_pose_2.header.frame_id = 'map'
        goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_2.pose.position.x = -0.3033531904220581
        goal_pose_2.pose.position.y = -0.4876289367675781
        goal_pose_2.pose.orientation.z = 0.450693383506524
        goal_pose_2.pose.orientation.w = 0.892678819096455
        goals.append(goal_pose_2)

        goal_pose_3 = PoseStamped()
        goal_pose_3.header.frame_id = 'map'
        goal_pose_3.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_3.pose.position.x = 0.017118453979492188
        goal_pose_3.pose.position.y = 0.015325784683227539
        goal_pose_3.pose.orientation.z = -0.26655083137572366
        goal_pose_3.pose.orientation.w = 0.9638208621382454
        goals.append(goal_pose_3)

        goal_pose_4 = PoseStamped()
        goal_pose_4.header.frame_id = 'map'
        goal_pose_4.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_4.pose.position.x = 0.5769392577441993
        goal_pose_4.pose.position.y = -0.43399156158840846
        goal_pose_4.pose.orientation.z = -0.8272857052020762
        goal_pose_4.pose.orientation.w = 0.5617814183188186
        goals.append(goal_pose_4)


def main(args=None):
    rp.init(args=args)
    
    navi_qt = navi_subscriber()
    rp.spin(navi_qt)
    navi_qt.navigator.lifecycleShutdown()
    navi_qt.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
