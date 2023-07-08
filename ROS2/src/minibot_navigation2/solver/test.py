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
    initial_pose.pose.position.x = -0.007758932573928677
    initial_pose.pose.position.y = 0.6301214142716467
    initial_pose.pose.orientation.z = -0.6673251397576241
    initial_pose.pose.orientation.w = 0.7447665123026596
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Define the waypoints manually
    waypoints = [
        (0.0771390453055478, -0.03556606702952142, -0.6845749077980176, 0.7289425187306169),
        (0.0531927779739269, 0.49975126994125885, -0.729984417648241, 0.683463788353676),
        (0.45526618913384936, -2.1973543766605528, 0.777120276737929, 0.6293521077129755),
        (0.0531927779739269, 0.49975126994125885, -0.729984417648241, 0.683463788353676)
    ]

    for waypoint in waypoints:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoint[0]
        goal_pose.pose.position.y = waypoint[1]
        goal_pose.pose.orientation.z = waypoint[2]
        goal_pose.pose.orientation.w = waypoint[3]
        goals.append(goal_pose)

    i = 0
    for goal in goals:
        i += 1
        navigator.goThroughPoses([goal])  # Move to the current goal

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
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