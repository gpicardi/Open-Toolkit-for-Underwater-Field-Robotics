import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from time import sleep
import board
from adafruit_ms8607 import MS8607

class Ms8607Node(Node):

	def __init__(self):
		# Create i2c and sensor objects
		self.i2c = board.I2C()
		self.sensor = MS8607(self.i2c)
		
		# Initialize parent (ROS Node)
		super().__init__('ms8607')
		self.pressure_publisher_ = self.create_publisher(Float64, '/ms8607/pressure', 10)
		self.temperature_publisher_ = self.create_publisher(Float64, '/ms8607/temperature', 10)
		self.humidity_publisher_ = self.create_publisher(Float64, '/ms8607/humidity', 10)
		timer_period = 0.5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		
	def timer_callback(self):
		pressure_msg = Float64()
		pressure_msg.data = self.sensor.pressure
		
		temperature_msg = Float64()
		temperature_msg.data = self.sensor.temperature
		
		humidity_msg = Float64()
		humidity_msg.data = self.sensor.relative_humidity
		
		self.pressure_publisher_.publish(pressure_msg)
		self.temperature_publisher_.publish(temperature_msg)
		self.humidity_publisher_.publish(humidity_msg)
		
		#self.get_logger().info("Publishing Pressure: %.2f" %pressure_msg.data)
		#self.get_logger().info("Publishing Temperature: %.2f" %temperature_msg.data)
		#self.get_logger().info("Publishing Humidity: %.2f" %humidity_msg.data)
		self.i +=1

def main(args=None):
	rclpy.init(args=args)
	ms8607_publisher = Ms8607Node()
	rclpy.spin(ms8607_publisher)
	
	ms8607_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
