#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Int64

import time


class ForwardPositionPublisherSilver(Node):

    def __init__(self):
        super().__init__('long_test_publisher_silver')
        self.motor_cmd_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        timer_period = 5  # seconds
        self.motor_timer = self.create_timer(timer_period, self.motor_callback)
        self.motor_cmds = np.array([
            [0.0, -1.3, 0.0, 0.0, -1.3, 0.0, 0.0, -1.3, 0.0, 0.0, -1.3, 0.0, 0.0, -1.3, 0.0],
            [0.2, -1.5, 0.2, 0.2, -1.5, 0.2, -0.2, -1.5, 0.2, -0.2, -1.5, 0.2, -0.2, -1.5, 0.2],
            [0.0, -1.3, 0.0, 0.0, -1.3, 0.0, 0.0, -1.3, 0.0, 0.0, -1.3, 0.0, 0.0, -1.3, 0.0], 
            [-0.2, -1.3, 0.2, -0.2, -1.3, 0.2, 0.2, -1.3, 0.2, 0.2, -1.3, 0.2, 0.2, -1.3, 0.2]
            ])
        self.i = 0
        self.N = 4

        self.torches_timer = self.create_timer(timer_period, self.torches_callback)
        self.torches_publisher = self.create_publisher(Int64, 'torches_intensity', 10)
        self.torches_cmds = np.array([100, 0])


    def motor_callback(self):
        msg = Float64MultiArray()
        tmp =  self.motor_cmds[self.i%self.N]
        tmp = tmp.tolist()
        msg.data = tmp
        self.motor_cmd_publisher.publish(msg)
        self.i += 1

    def torches_callback(self):
        msg = Int64()
        msg.data = 100
        self.torches_publisher.publish(msg)
        time.sleep(1)
        msg.data = 0
        self.torches_publisher.publish(msg)


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
