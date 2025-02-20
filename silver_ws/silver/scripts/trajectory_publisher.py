#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LegTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('leg_trajectory_publisher')

        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['coxa_joint','femur_joint','tibia_joint']
        self.i = 0

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
        p3.positions=[0.785, 0.785, 0.785]
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

    def timer_callback(self):
        msg = JointTrajectory()
        #msg.header.stamp = self.get_clock().now().to_msg()
        #msg.header.frame_id = ''
        msg.joint_names =['coxa_joint', 'femur_joint', 'tibia_joint']
        msg.points.append(self.points[self.i%4])
        print(msg)
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    leg_trajectory_publisher = LegTrajectoryPublisher()

    rclpy.spin(leg_trajectory_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leg_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()