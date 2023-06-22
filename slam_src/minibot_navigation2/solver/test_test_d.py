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
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

def main():
    rclpy.init()
    goals=[]
    navigator = BasicNavigator()
    # to_radian = np.pi / 180.
    # tmp_in = [-0.05100677162408829, 0.49727797508239746, 0.11572265625]
    # target1 = quaternion_from_euler(tmp_in[0] * to_radian, tmp_in[1] * to_radian, tmp_in[2] * to_radian)
    # tmp_g1 = [0.086948461830616, -0.21363921463489532, 0.11578369140625]
    # target2 = quaternion_from_euler(tmp_g1[0] * to_radian, tmp_g1[1] * to_radian, tmp_g1[2] * to_radian)
    # tmp_g2 = [0.3296409547328949, -2.142056465148926, 0.002471923828125]
    # target3 = quaternion_from_euler(tmp_g2[0] * to_radian, tmp_g2[1] * to_radian, tmp_g2[2] * to_radian)

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -1.6781088109157032
    initial_pose.pose.position.y = 0.585551510681771
    initial_pose.pose.orientation.z = -0.2792168827477615
    initial_pose.pose.orientation.w = 0.9602280626958487
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -0.9990098476409912
    goal_pose.pose.position.y = 0.057299137115478516
    goal_pose.pose.orientation.z = -0.28940227799286045
    goal_pose.pose.orientation.w = 0.9572075644772888
    goals.append(goal_pose)

    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x = -0.3033531904220581
    goal_pose_1.pose.position.y = -0.4876289367675781
    goal_pose_1.pose.orientation.z = 0.450693383506524
    goal_pose_1.pose.orientation.w = 0.892678819096455
    goals.append(goal_pose_1)

    goal_pose_2 = PoseStamped()
    goal_pose_2.header.frame_id = 'map'
    goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_2.pose.position.x = 0.017118453979492188
    goal_pose_2.pose.position.y = 0.015325784683227539
    goal_pose_2.pose.orientation.z = -0.26655083137572366
    goal_pose_2.pose.orientation.w = 0.9638208621382454
    goals.append(goal_pose_2)

    goal_pose_3 = PoseStamped()
    goal_pose_3.header.frame_id = 'map'
    goal_pose_3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_3.pose.position.x = 0.5769392577441993
    goal_pose_3.pose.position.y = -0.43399156158840846
    goal_pose_3.pose.orientation.z = -0.8272857052020762
    goal_pose_3.pose.orientation.w = 0.5617814183188186
    goals.append(goal_pose_3)

    i = 0
    for goal in goals:
        i += 1
        navigator.goThroughPoses([goal])  # Move to the current goal

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                navigator.cancelTask()

        print('Moving towards goal pose {}'.format(i))
        print('g{}'.format(i))



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

    navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()
