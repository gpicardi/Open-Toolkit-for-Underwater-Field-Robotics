#!/usr/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .ms5837api import *
import time

class Ms5837Node(Node):
    
    def __init__(self, bus=1):        
        #create sensor object on selected bus
        self.sensor = MS5837_30BA(bus)
        self.sensor.init()
        self.sensor.setFluidDensity(DENSITY_SALTWATER)

        if not self.sensor.read():
            print("Sensor read failed!")
            exit(1)

        # Initialize parent (ROS Node)
        super().__init__('ms5837')
        self.pressure_publisher_ = self.create_publisher(Float64, '/ms5837/pressure', 10)
        self.temperature_publisher_ = self.create_publisher(Float64, '/ms5837/temperature', 10)
        self.depth_publisher_ = self.create_publisher(Float64, '/ms5837/depth', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.get_logger().info("Sensor MS5837 on bus i2c-1 correctly initialized.")

    def timer_callback(self):
        pressure_msg = Float64()
        temperature_msg = Float64()
        depth_msg = Float64()

        try:
            self.sensor.read()
            pressure_msg.data = self.sensor.pressure()
            temperature_msg.data = self.sensor.temperature()
            depth_msg.data = self.sensor.depth()

            self.pressure_publisher_.publish(pressure_msg)
            self.temperature_publisher_.publish(temperature_msg)
            self.depth_publisher_.publish(depth_msg)
            
            self.i +=1
        except:
            self.i +=1


        """self.get_logger().info("Publishing Pressure: %.2f mbar" %pressure_msg.data)
        self.get_logger().info("Publishing Temperature: %.2f C" %temperature_msg.data)
        self.get_logger().info("Publishing Depth in Saltwater: %.7f m" %depth_msg.data)"""

def main(args=None):
    rclpy.init(args=args)
    ms5837_publisher = Ms5837Node()
    rclpy.spin(ms5837_publisher)

    ms5837_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()