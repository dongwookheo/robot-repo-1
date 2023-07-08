#! /usr/bin/env python3
'''
This is a Python script that uses the nav2_simple_commander package to navigate a robot
through a sequence of pre-defined poses in a ROS 2 environment. The BasicNavigator class
is used to control the robot's movement, and the PoseStamped class is used to represent each goal pose.
The setInitialPose() method is used to set the robot's initial position, and goThroughPoses() is used to navigate
the robot through the sequence of goals.
 The script waits for the task to complete before printing the result and shutting down the navigator.
'''
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import numpy as np
import threading
import sys
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Int32, String

class GoalSubscriber:
    def __init__(self):
        self.goal_id = None
        self.table_list = None
        self.subscriber = None
        self.subscriber1 = None
        self.node = None

    def callback(self, msg):
        if isinstance(msg, Int32):
            self.goal_id = msg.data
        if isinstance(msg, String):
            self.table_list = msg.data
        print(self.goal_id)
        print(self.table_list)

    def subscribe(self):
        #rclpy.init()
        self.node = rclpy.create_node('goal_subscriber')
        self.subscriber = self.node.create_subscription(Int32, 'aruco_id', self.callback, 10)
        self.subscriber  # prevent unused variable warning
        self.subscriber1 = self.node.create_subscription(String, 'table_topic', self.callback, 10)
        self.subscriber1  # prevent unused variable warning
        
    def get_goal_id(self):
        return self.goal_id
    
    def get_table_list(self):
        return self.table_list
    
def setInitialPose(navigator):
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -1.6781088109157032
    initial_pose.pose.position.y = 0.585551510681771
    initial_pose.pose.orientation.z = -0.2792168827477615
    initial_pose.pose.orientation.w = 0.9602280626958487
    navigator.setInitialPose(initial_pose)

def setNavigationGoals_0(navigator, goals):
    goal_pose_0 = PoseStamped()
    goal_pose_0.header.frame_id = 'map'
    goal_pose_0.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_0.pose.position.x = -1.6781088109157032
    goal_pose_0.pose.position.y = 0.585551510681771
    goal_pose_0.pose.orientation.z = -0.2792168827477615
    goal_pose_0.pose.orientation.w = 0.9602280626958487
    goals.append(goal_pose_0)

def setNavigationGoals_1(navigator, goals):
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

def main():
    while True:
        rclpy.init()
        goal_subscriber = GoalSubscriber()
        
        navigator = BasicNavigator()
        goals=[]
        setInitialPose(navigator)
        navigator.waitUntilNav2Active() #네비게이션 시스템이 초기화된 후 동작 가능한 상태가 될 때까지 대기
        
        # Run the goal subscriber in a separate thread
        subscriber_thread = threading.Thread(target=goal_subscriber.subscribe)
        subscriber_thread.start()   
        
        goal_id = goal_subscriber.get_goal_id()
        table_list = goal_subscriber.get_table_list()

        if table_list == "a_table": 
            setNavigationGoals_1(navigator, goals) # 1번 경로 설정
        #rclpy.spin_once(goal_subscriber.node, timeout_sec=0.1)

        if goal_id == 1 or goal_id == 23:
            setNavigationGoals_0(navigator, goals)
 #######################################################################
        i = 0
        
        for goal in goals:
            i += 1
            
            navigator.goThroughPoses([goal]) # Move to the current goal
            print([goal])
    
            while rclpy.ok() and not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
                    navigator.cancelTask()
                if feedback.distance_remaining < 0.2 or Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
                    break
                
            print('Moving towards goal pose {}'.format(i))
            print('g{}'.format(i))
    
            # 주행이 완료될 때까지 메시지 처리를 수행
            while not navigator.isTaskComplete():
                rclpy.spin_once(goal_subscriber.node, timeout_sec=0.1)
    
        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
    
################## nav2 종료  #################################
        navigator.lifecycleShutdown()
        subscriber_thread.join()
    
        choice = input('다른 주행 명령을 실행하시겠습니까? (y/n): ')
        if choice.lower() == 'n':
            break
        rclpy.shutdown()    
    
if __name__ == '__main__':
    main()  