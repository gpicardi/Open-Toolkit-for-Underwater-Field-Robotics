#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class ForwardPositionPublisherSilver(Node):

    def __init__(self):
        super().__init__('forward_position_publisher_silver')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmds = np.array([
            [0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0],
            [0.5, 2.5, -2.5, 0.5, 2.5, -2.5, 0.5, 2.5, -2.5, 0.5, 2.5, -2.5, 0.5, 2.5, -2.5, 0.5, 2.5, -2.5],
            [0.0, 2.0, -2.5, 0.0, 2.0, -2.5, 0.0, 2.0, -2.5, 0.0, 2.0, -2.5, 0.0, 2.0, -2.5, 0.0, 2.0, -2.5], 
            [-0.5, 2.5, -2.0, -0.5, 2.5, -2.0, -0.5, 2.5, -2.0, -0.5, 2.5, -2.0, -0.5, 2.5, -2.0, -0.5, 2.5, -2.0],
            [0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0],
            [0.5, 2.0, -2.0, 0.5, 2.0, -2.0, 0.5, 2.0, -2.0, 0.5, 2.0, -2.0, 0.5, 2.0, -2.0, 0.5, 2.0, -2.0],
            [-0.5, 2.0, -2.0, -0.5, 2.0, -2.0, -0.5, 2.0, -2.0, -0.5, 2.0, -2.0, -0.5, 2.0, -2.0, -0.5, 2.0, -2.0],
            [0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0, 0.0, 2.0, -2.0]
            ])
        self.i = 0
        self.N = 4

    def timer_callback(self):
        msg = Float64MultiArray()
        tmp =  self.cmds[self.i%self.N]
        tmp = tmp.tolist()
        msg.data = tmp
        print(tmp)
        self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    fpp = ForwardPositionPublisherSilver()

    rclpy.spin(fpp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fpp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()