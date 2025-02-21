import rclpy
from rclpy.node import Node
from silver_interfaces.msg import DhtSensorData  # Import your custom message
import adafruit_dht
import board
import time

class DHT11SensorNode(Node):
    def __init__(self):
        super().__init__('dht11_sensor_node')

        self.declare_parameter('gpio_pin', 4)  # Default to GPIO 4
        self.declare_parameter('sensor_location', 'Unknown')
        self.declare_parameter('sensor_type', 'DHT11')

        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.sensor_location = self.get_parameter('sensor_location').value
        self.sensor_type = self.get_parameter('sensor_type').value

        gpio_pins = [
            board.D4, board.D5, board.D6, board.D7, board.D9, board.D10, board.D11, board.D13,
            board.D14, board.D15, board.D16, board.D17, board.D20, board.D21, board.D22, board.D23,
            board.D24, board.D25, board.D26, board.D27
            ]

        sensor_locations = [
            'F3', 'T1', 'F1', 'F4', 'C5', 'F5', 'ControlC', 'CameraC', 'C3', 'T4', 'T3', 'T2',
            'C4', 'T5', 'C2', 'T0', 'F2', 'C0', 'C1', 'F0'
        ]

        sensor_gpio_mapping = dict(zip(sensor_locations, gpio_pins))
        
        # New ADAFRUIT Code
        self.dhtDevice = adafruit_dht.DHT11(sensor_gpio_mapping.get(self.sensor_location))

        # Create publisher for the custom message
        self.sensor_data_publisher = self.create_publisher(DhtSensorData, 'dht11_sensor_data', 10)

        # Timer to read the sensor data periodically
        self.timer = self.create_timer(2.0, self.new_read_sensor_data)  # 2-second interval

        self.get_logger().info("DHT11 sensor node has started.")

    """def read_sensor_data(self):
        humidity, temperature = Adafruit_DHT.read_retry(self.sensor, self.gpio_pin)

        if humidity is not None and temperature is not None:
            self.get_logger().info(f"GPIO: {self.gpio_pin}, Temperature: {temperature:.2f} °C, Humidity: {humidity:.2f} %")

            # Publish the custom message
            sensor_data_msg = DhtSensorData()
            sensor_data_msg.location = self.sensor_location
            sensor_data_msg.temperature = temperature
            sensor_data_msg.humidity = humidity
            self.sensor_data_publisher.publish(sensor_data_msg)
        else:
            self.get_logger().warning("Failed to read data from DHT11 sensor.")"""

    def new_read_sensor_data(self):

        while True:
            try:
                # Print the values to the serial port
                temperature_c = self.dhtDevice.temperature
                humidity = self.dhtDevice.humidity
                print(
                    "Temp: {:.1f} C    Humidity: {}% ".format(
                        temperature_c, humidity
                    )
                )
                # Publish the custom message
                sensor_data_msg = DhtSensorData()
                sensor_data_msg.location = self.sensor_location
                sensor_data_msg.temperature = float(temperature_c)
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

            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)

    node = DHT11SensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

