import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity
from silver_interfaces.msg import DhtSensorData  
import adafruit_dht
import board
import time

class DHT11Publisher(Node):
    def __init__(self):
        super().__init__('dht11_publisher')
        
        # Define GPIO pins for the three sensors
        self.sensor_pins = [board.D23, board.D25, board.D27] 
        self.sensor_location = ['T0','C0','F0']
        self.devices = [adafruit_dht.DHT11(pin) for pin in self.sensor_pins]
        
        # Create publisher for the custom message
        self.sensor_data_publisher = self.create_publisher(DhtSensorData, 'dht11_sensor_data', 10)
        self.dhtDevice = adafruit_dht.DHT11(board.D21)
        
        # Create a timer to read and publish data sequentially
        self.timer = self.create_timer(2.0, self.read_sensors)  # Read every 2 seconds
        
    def read_sensors(self):
        for i, sensor in enumerate(self.devices):
            try:
                # Print the values to the serial port
                temperature = sensor.temperature
                humidity = sensor.humidity
                print(
                    "Gpio: {} Temp: {:.1f} C    Humidity: {}% ".format(
                        self.sensor_location[i], temperature, humidity
                    )
                )
                # Publish the custom message
                sensor_data_msg = DhtSensorData()
                sensor_data_msg.location = self.sensor_location[i]
                sensor_data_msg.temperature = float(temperature)
                sensor_data_msg.humidity = float(humidity)
                self.sensor_data_publisher.publish(sensor_data_msg)

            except RuntimeError as error:
                # Errors happen fairly often, DHT's are hard to read, just keep going
                print(error.args[0])
                time.sleep(2.0)
                continue

            except Exception as error:
                self.dhtDevice.exit()
                raise error
            
            time.sleep(1)  # Small delay between sensor readings
        

def main(args=None):
    rclpy.init(args=args)
    node = DHT11Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()