import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from time import sleep
import Adafruit_DHT

class Dht11Node(Node):

	def __init__(self):
		# Create sensor object and specify input pin (Repeat for 6 sensors)
		self.DHT_SENSOR = Adafruit_DHT.DHT11
		self.DHT_PINS = [6] #[4, 5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27]
		# Initialize parent (ROS Node)
		super().__init__('dht11')
		# What data structure for 6 sensors? Custom or Float64Multiarrray?
		self.temperature_publisher_ = self.create_publisher(Float64MultiArray, '/dht11/temperature', 10)
		self.humidity_publisher_ = self.create_publisher(Float64MultiArray, '/dht11/humidity', 10)
		timer_period = 1
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

		self.get_logger().info("Sensors DHT11 correctly initialized.")
		
	def timer_callback(self):
		# Create data structure for the publishers
		temperature_msg = Float64MultiArray()
		humidity_msg = Float64MultiArray()
		temperature = []
		humidity = []

		# Read data from sensors and append to messages
		for pin in self.DHT_PINS:
			humidity_data, temperature_data = Adafruit_DHT.read(self.DHT_SENSOR, pin)
			if temperature_data == None:
				temperature.append(-1.0)
			else:
				temperature.append(temperature_data)
			if humidity_data == None:
				humidity.append(-1.0)
			else:
				humidity.append(humidity_data)

		temperature_msg.data = temperature
		humidity_msg.data = humidity
		self.temperature_publisher_.publish(temperature_msg)
		self.humidity_publisher_.publish(humidity_msg)
		self.i +=1

def main(args=None):
	rclpy.init(args=args)
	dht11_publisher = Dht11Node()
	rclpy.spin(dht11_publisher)
	
	dht11_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
