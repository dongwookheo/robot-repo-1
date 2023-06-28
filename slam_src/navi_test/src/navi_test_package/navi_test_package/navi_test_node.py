import rclpy as rp
import time
from rclpy.node import Node
from std_msgs.msg import String, Int32, UInt16
from geometry_msgs.msg import Twist, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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
        self.subscription1 = self.create_subscription(
            Int32, 
            'aruco_id',
            self.callback1,
            10
        )
        self.subscription2 = self.create_subscription(
            UInt16,
            '/depth',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/base_controller/cmd_vel_unstamped',
            10
        )

        self.goals = []
        self.navigator = BasicNavigator()
        self.setInitialPose(self.navigator)

    def listener_callback(self, msg):
        depth = msg.data
        print(f'Received depth: {depth}')
        if depth > 51:
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.5  # 전진 속도 설정 (여기서는 0.2m/s)
            twist_cmd.angular.z = 0.0  # 회전 속도 설정 (여기서는 0으로 설정하여 직진)
            self.publisher.publish(twist_cmd)  # 명령을 발행하여 로봇을 전진시킴
        else :
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.0  # 전진 속도 설정 (여기서는 0.2m/s)
            twist_cmd.angular.z = 0.0  # 회전 속도 설정 (여기서는 0으로 설정하여 직진)
            self.publisher.publish(twist_cmd)  # 명령을 발행하여 로봇을 전진시킴

    def callback(self, msg) :
        self.goals = []
        #self.navigator = BasicNavigator()
        #self.setInitialPose(self.navigator)
        self.navigator.waitUntilNav2Active()
        print('네비게이션 준비 중')
        if msg.data == "a table" :
            print("a 경로 설정 됨")
            self.setNavigationGoals_1(self.navigator, self.goals) # 1번 경로 설정
        if msg.data == "b table" :
            print("b 경로 설정 됨")
            self.setNavigationGoals_2(self.navigator, self.goals) # 2번 경로 설정
        if msg.data == "initial position" :
            print("초기 경로 설정 됨")
            self.setNavigationGoals_0(self.navigator, self.goals) # 1번 경로 설정

        i = 0
        for goal in self.goals:
            i += 1
            print("이동 중 입니다.")
            self.navigator.goThroughPoses([goal])  # Move to the current goal

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))
                if feedback.distance_remaining < 0.1 or Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
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
        #self.navigator.cancelTask()

    def callback1(self, msg) :
        if msg.data == 1:
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.5  # 전진 속도 설정 (여기서는 0.2m/s)
            twist_cmd.angular.z = 0.0  # 회전 속도 설정 (여기서는 0으로 설정하여 직진)
            self.publisher.publish(twist_cmd)  # 명령을 발행하여 로봇을 전진시킴
            # 전진을 위한 코드 추가 끝

            start_time = time.time()  # 시작 시간 저장
            while time.time() - start_time < 3.0:  # 30초 동안 반복
                self.publisher.publish(twist_cmd)
                time.sleep(0.1)  # 매 0.1초마다 Twist 명령을 전송합니다.
            twist_cmd.linear.x = 0.0  # 전진 멈춤
            self.publisher.publish(twist_cmd)
        
        if msg.data == 23:
            twist_cmd = Twist()
            twist_cmd.linear.x = 0.5  # 전진 속도 설정 (여기서는 0.2m/s)
            twist_cmd.angular.z = 0.0  # 회전 속도 설정 (여기서는 0으로 설정하여 직진)
            self.publisher.publish(twist_cmd)  # 명령을 발행하여 로봇을 전진시킴
            # 전진을 위한 코드 추가 끝

            start_time = time.time()  # 시작 시간 저장
            while time.time() - start_time < 5.0:  # 30초 동안 반복
                self.publisher.publish(twist_cmd)
                time.sleep(0.1)  # 매 0.1초마다 Twist 명령을 전송합니다.
            twist_cmd.linear.x = 0.0  # 전진 멈춤
            self.publisher.publish(twist_cmd)

    #def callback1(self, msg) :
    #    self.goals = []
    #    self.navigator = BasicNavigator()
    #    self.setInitialPose(self.navigator)
    #    self.navigator.waitUntilNav2Active()
    #    if msg.data == 23 :
    #        self.setNavigationGoals_0(self.navigator, self.goals) # 1번 경로 설정
    #    if msg.data == 1 :
    #        self.setNavigationGoals_0(self.navigator, self.goals) # 2번 경로 설정
    #
    #    i = 0
    #    for goal in self.goals:
    #        i += 1
    #        self.navigator.goThroughPoses([goal])  # Move to the current goal
#
    #        while not self.navigator.isTaskComplete():
    #            feedback = self.navigator.getFeedback()
    #            if feedback and i % 5 == 0:
    #                print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))
    #            if feedback.distance_remaining < 0.1 or Duration.from_msg(feedback.navigation_time) > Duration(seconds=40.0):
    #                break
#
    #        print('Moving towards goal pose {}'.format(i))
    #        print('g{}'.format(i))
#
    #    # Do something depending on the return code
    #    result = self.navigator.getResult()
    #    
    #    if result == TaskResult.SUCCEEDED:
    #        print('Goal succeeded!')
    #    elif result == TaskResult.CANCELED:
    #        print('Goal was canceled!')
    #    elif result == TaskResult.FAILED:
    #        print('Goal failed!')
    #    else:
    #        print('Goal has an invalid return status!')
    #    #self.navigator.cancelTask()

    def setInitialPose(self, navigator):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -0.012410423149555823
        initial_pose.pose.position.y = 0.006677273066386658
        initial_pose.pose.orientation.z = 0.0018261075243604478
        initial_pose.pose.orientation.w = 0.9999983326642647
        navigator.setInitialPose(initial_pose)

    def setNavigationGoals_0(self, navigator, goals):
        goal_pose_0 = PoseStamped()
        goal_pose_0.header.frame_id = 'map'
        goal_pose_0.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_0.pose.position.x = -0.012410423149555823
        goal_pose_0.pose.position.y = 0.006677273066386658
        goal_pose_0.pose.orientation.z = 0.0018261075243604478
        goal_pose_0.pose.orientation.w = 0.9999983326642647
        goals.append(goal_pose_0)

    def setNavigationGoals_1(self, navigator, goals):
        goal_pose_1 = PoseStamped()
        goal_pose_1.header.frame_id = 'map'
        goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_1.pose.position.x = 1.1321052943158862
        goal_pose_1.pose.position.y = 0.5113690474174074
        goal_pose_1.pose.orientation.z = 0.010712174708092168
        goal_pose_1.pose.orientation.w = 0.9999426230104522
        goals.append(goal_pose_1)

        goal_pose_2 = PoseStamped()
        goal_pose_2.header.frame_id = 'map'
        goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_2.pose.position.x = 1.1601511481199875
        goal_pose_2.pose.position.y = 0.4549994322175879
        goal_pose_2.pose.orientation.z = -0.7085567829610658
        goal_pose_2.pose.orientation.w = 0.7056537999046452
        goals.append(goal_pose_2)

    def setNavigationGoals_2(self, navigator, goals):
        goal_pose_1 = PoseStamped()
        goal_pose_1.header.frame_id = 'map'
        goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_1.pose.position.x = 1.1321052943158862
        goal_pose_1.pose.position.y = 0.5113690474174074
        goal_pose_1.pose.orientation.z = 0.010712174708092168
        goal_pose_1.pose.orientation.w = 0.9999426230104522
        goals.append(goal_pose_1)

        goal_pose_2 = PoseStamped()
        goal_pose_2.header.frame_id = 'map'
        goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_2.pose.position.x = 2.4051413345510544
        goal_pose_2.pose.position.y = 0.4437311457767596
        goal_pose_2.pose.orientation.z = -0.7025926138990058
        goal_pose_2.pose.orientation.w = 0.711592312278992
        goals.append(goal_pose_2)

    def setNavigationGoals_3(self, navigator, goals):
        goal_pose_1 = PoseStamped()
        goal_pose_1.header.frame_id = 'map'
        goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_1.pose.position.x = 0.5272699040990709
        goal_pose_1.pose.position.y = -0.7318250826923461
        goal_pose_1.pose.orientation.z = 0.02119517002034199
        goal_pose_1.pose.orientation.w = 0.9997753571516997
        goals.append(goal_pose_1)

        goal_pose_2 = PoseStamped()
        goal_pose_2.header.frame_id = 'map'
        goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_2.pose.position.x = 1.1628363127029528
        goal_pose_2.pose.position.y = -0.7526577759573901
        goal_pose_2.pose.orientation.z = 0.7136218595127126
        goal_pose_2.pose.orientation.w = 0.7005311139597
        goals.append(goal_pose_2)

    def setNavigationGoals_4(self, navigator, goals):
        goal_pose_1 = PoseStamped()
        goal_pose_1.header.frame_id = 'map'
        goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_1.pose.position.x = 0.6886391639709473
        goal_pose_1.pose.position.y = 0.2992539405822754
        goal_pose_1.pose.orientation.z = 0.01372770470826775
        goal_pose_1.pose.orientation.w = 0.9999057706221335
        goals.append(goal_pose_1)

        goal_pose_2 = PoseStamped()
        goal_pose_2.header.frame_id = 'map'
        goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_2.pose.position.x = 1.7522367238998413
        goal_pose_2.pose.position.y = 0.3783102035522461
        goal_pose_2.pose.orientation.z = -0.7113450660322955
        goal_pose_2.pose.orientation.w = 0.7028429390920201
        goals.append(goal_pose_2)

        goal_pose_3 = PoseStamped()
        goal_pose_3.header.frame_id = 'map'
        goal_pose_3.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_3.pose.position.x = 2.253790855407715
        goal_pose_3.pose.position.y = -0.8882979154586792
        goal_pose_3.pose.orientation.z = 0.7073881459730432
        goal_pose_3.pose.orientation.w = 0.7068253043976429
        goals.append(goal_pose_3)


def main(args=None):
    rp.init(args=args)
    
    navi_qt = navi_subscriber()
    rp.spin(navi_qt)
    navi_qt.navigator.lifecycleShutdown()
    navi_qt.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
