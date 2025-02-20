#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class ForwardPositionPublisher(Node):

    def __init__(self):
        super().__init__('forward_position_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.trj = [[0.0, 0.0, 0.0],[0.1, 0.1, 0.1],[0.2, 0.2, 0.2], [0.3, 0.3, 0.3],[0.4, 0.4, 0.4], [0.3, 0.3, 0.3],[0.2, 0.2, 0.2], [0.1, 0.1, 0.1]]
        self.trj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]]
        #self.trj = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]]
        self.i = 0

       

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.trj[self.i%2]
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    forward_position_publisher = ForwardPositionPublisher()

    rclpy.spin(forward_position_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    forward_position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()