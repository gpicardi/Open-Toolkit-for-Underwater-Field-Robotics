#!/usr/bin/env python3

import time
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class FollowPositionTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('follow_position_trajectory_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_position_controller/follow_joint_trajectory')

        if len(sys.argv)==2:
            self.index = int(sys.argv[1])
        else:
            print("wrong number of parameters")
            return

        p1 = JointTrajectoryPoint()
        p1.positions=[0.785, 0.785, 0.785]
        p1.velocities=[]
        p1.accelerations=[]
        p1.effort=[]
        p1.time_from_start=rclpy.time.Duration(seconds=2).to_msg()

        p2 = JointTrajectoryPoint()
        p2.positions=[1.57, 1.57, 1.57]
        p2.velocities=[]
        p2.accelerations=[]
        p2.effort=[]
        p2.time_from_start=rclpy.time.Duration(seconds=2).to_msg()

        p3 = JointTrajectoryPoint()
        p3.positions=[-0.785, -0.785, -0.785]
        p3.velocities=[]
        p3.accelerations=[]
        p3.effort=[]
        p3.time_from_start=rclpy.time.Duration(seconds=2).to_msg()

        p4 = JointTrajectoryPoint()
        p4.positions=[0.0, 0.0, 0.0]
        p4.velocities=[]
        p4.accelerations=[]
        p4.effort=[]
        p4.time_from_start=rclpy.time.Duration(seconds=2).to_msg()

        self.points = [p1, p2, p3, p4]


    def send_goal(self, index):
        # I am ignoring the parameter from outside to use the one passed from command line. To be adapted to the use
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names =['coxa_joint', 'femur_joint', 'tibia_joint']
        goal_msg.trajectory.points.append(self.points[self.index])

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.error_code))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.actual))


def main(args=None):
    rclpy.init(args=args)

    action_client = FollowPositionTrajectoryActionClient()

    """for index in range(0,4):
        input('press any button to send the next goal')
        action_client.send_goal(index)
        time.sleep(1)"""
    
    action_client.send_goal(0)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()