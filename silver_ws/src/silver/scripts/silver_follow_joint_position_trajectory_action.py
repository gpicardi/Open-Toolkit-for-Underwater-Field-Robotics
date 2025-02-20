#!/usr/bin/env python3

import numpy as np

import leg_math

import rclpy
from rclpy.time import Time, Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class SilverFollowPositionTrajectoryAction(Node):

    def __init__(self):
        # ROS stuff
        super().__init__('follow_position_trajectory_action') #initialize ros Node
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_position_controller/follow_joint_trajectory') #initialize action client for the type FollowJointTrajectory and declare the output topic
        self.subscription = self.create_subscription(Float32, '/direction', self.listener_callback, 10) #initialize the subscriber, listening on the topic /direction
        self.subscription
        #Class objects
        self.cc_listener = 0
        self.dir = 0 #direction, needed for steering the gait, updated by the listener callback
        self.dt = 0.4 #time step for trajectory waypoints
        self.dt_duration = Duration(seconds=self.dt) # use duration instead of float 
        self.N = 10 #number of waypoints in a trajectory (self.dt*self.N = GAIT PERIOD)
        self.leg = leg_math.LegMath() #contains fw_kine and inv_kine to generate trajectories
        #Goal settings
        self.goal_msg = FollowJointTrajectory.Goal() #initialize Goal, updated by create_trajectory functions
        self.goal_msg.trajectory.joint_names =['coxa_joint_0', 'femur_joint_0', 'tibia_joint_0',
                                          'coxa_joint_1', 'femur_joint_1', 'tibia_joint_1',
                                          'coxa_joint_2', 'femur_joint_2', 'tibia_joint_2',
                                          'coxa_joint_3', 'femur_joint_3', 'tibia_joint_3',
                                          'coxa_joint_4', 'femur_joint_4', 'tibia_joint_4',
                                          'coxa_joint_5', 'femur_joint_5', 'tibia_joint_5']
        self.goal_msg.trajectory.header.stamp = Time(seconds=0.0).to_msg()

    def create_simple_trajectory(self): # create a simple trajectory for debugging purposes
        
        p0 = JointTrajectoryPoint()
        p0.positions = 18*[1.0]
        p0.velocities=[]
        p0.accelerations=[]
        p0.effort=[]
        p0.time_from_start=rclpy.time.Duration(seconds=1.0).to_msg()
        self.goal_msg.trajectory.points.append(p0)

        p1 = JointTrajectoryPoint()
        p1.positions = 18*[0.0]
        p1.velocities=[]
        p1.accelerations=[]
        p1.effort=[]
        p1.time_from_start=rclpy.time.Duration(seconds=2.0).to_msg()
        self.goal_msg.trajectory.points.append(p1)

        p2 = JointTrajectoryPoint()
        p2.positions = 18*[1.5]
        p2.velocities=[]
        p2.accelerations=[]
        p2.effort=[]
        p2.time_from_start=rclpy.time.Duration(seconds=3.0).to_msg()
        self.goal_msg.trajectory.points.append(p2)

    def create_trajectory(self, leg_cart_trj): # create trajectories
        
        #create trj in cartesian space (for example an ellipsoid trajectory). In this all legs follow the same
        self.trj_cart_space = leg_cart_trj

        #convert trj to joint space for all legs and stack them. Dimensions: 18xN
        """self.trj_joint_space = np.hstack([
            np.vstack([self.leg.inv_kine(self.trj_cart_space[0:3, i], True)] * 6)
            for i in range(self.N)
        ])"""
        
        self.trj_joint_space = np.hstack([
            np.vstack((self.leg.inv_kine(self.trj_cart_space[0:3, i], True, False),
                      self.leg.inv_kine(self.trj_cart_space[0:3, i], True, False),
                      self.leg.inv_kine(self.trj_cart_space[0:3, i], True, False),
                      self.leg.inv_kine(self.trj_cart_space[0:3, i], True, True),
                      self.leg.inv_kine(self.trj_cart_space[0:3, i], True, True),
                      self.leg.inv_kine(self.trj_cart_space[0:3, i], True, True)))
            for i in range(self.N)
        ])

        #format waypoints using JointTrajectoryPoint obj and add them to the goal trajectory

        t_ = Time(seconds=0.0)

        """
        if self.cc_listener == 0:
            t_ = Time(seconds=0.0)
        else:
            t_last = self.goal_msg.trajectory.points[-1].time_from_start
            t_ = Time(seconds=(t_last).sec, nanoseconds=(t_last).nanosec)"""

        self.goal_msg.trajectory.points = []

        for i in range(0, self.N):
            t_ = t_ + self.dt_duration
            #t_ = t_ + self.dt
            p = JointTrajectoryPoint()
            p.positions = list(self.trj_joint_space[:,i])
            p.velocities=[]
            p.accelerations=[]
            p.effort=[]
            p.time_from_start=Duration(nanoseconds=t_.nanoseconds).to_msg()
            #p.time_from_start=Duration(seconds = t_).to_msg()
            self.goal_msg.trajectory.points.append(p)

        
    def send_goal(self):
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)

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
        #rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def listener_callback(self, msg): #this function should update the trajectories based on the user commands (only dir for now)
        self.create_trajectory(generate_ellipsoid_trajectory_Y(msg.data, 0.1, 0.1, 0.375, -0.3, self.N))
        self.send_goal()
        #self.get_logger().info('I heard: "%f"' % msg.data)
        self.cc_listener+=1


def generate_ellipsoid_trajectory_Y(A,B,x0=0, y0=0.375, z0 = 0, N = 10):
    t = np.linspace(0, 2*np.pi, N)
    x = A*np.cos(t-np.pi/2) + x0
    y = np.full(N,y0)
    z = B*np.sin(t-np.pi/2) + z0

    return np.array([x,y,z])

def generate_ellipsoid_trajectory_X(A,B,x0=0, y0=0.0, z0 = 0, N = 10):
    print(A)
    t = np.linspace(0, 2*np.pi, N)
    x = np.full(N,x0)
    y = A*np.cos(t-np.pi/2) + y0
    z = B*np.sin(t-np.pi/2) + z0

    return np.array([x,y,z])

def main(args=None):
    rclpy.init(args=args)
    action_client = SilverFollowPositionTrajectoryAction()
    #action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()