import rclpy
from rclpy.node import Node
from silver_interfaces.msg import DHTSensorData  # Import custom message
#import adafruit_dht
#import board
import Adafruit_DHT
import concurrent.futures


class DHT11Node(Node):
    def __init__(self):
        super().__init__('dht11_node')

        # Configuración
        #self.dht_sensor = adafruit_dht.DHT11(board.D25) #adafruit_dht
        self.dht_sensor = Adafruit_DHT.DHT11 #Adafruit_DHT
        self.gpio_pins = [4, 5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27]
        self.gpio_location = ['F4','T1','F1','F3','C5','F5','ControlC','CameraC','C4','T3','T4','T2','C3','T5','C2','T0','F0','C0','C1','F2']
        self.gpio_location_sorted = ['C0','F0','T0','C1','F1','T1','C2','F2','T2','C3','F3','T3','C4','F4','T4','C5','F5','T5','ControlC','CameraC']
        self.read_interval = 5  # Intervalo mínimo entre lecturas en segundos

        # Publicador de datos de todos los sensores
        self.sensor_data_publisher_ = self.create_publisher(DHTSensorData, '/dht11', 10)

        # Temporizador
        self.timer_ = self.create_timer(self.read_interval, self.read_sensors)

        self.get_logger().info('Nodo DHT11 inicializado. Publicando en /dht11/all_sensors_data.')

    def read_sensor(self, pin):
        """Lee temperatura y humedad de un sensor DHT11 en un GPIO específico."""
        try:
            #humidity = self.dhtDevice.humidity
            #temperature = self.dhtDevice.temperature
            humidity, temperature = Adafruit_DHT.read(self.dht_sensor, pin)
            if humidity is not None and temperature is not None:
                return temperature, humidity
            else:
                return -1.0, -1.0  # Indicador de error
        except Exception as e:
            self.get_logger().error(f"Error al leer el sensor en GPIO {pin}: {e}")
            return -1.0, -1.0  # Indicador de error

    def read_sensors(self):
        """Lee todos los sensores en paralelo y publica los datos."""
        temperatures = [-1.0] * len(self.gpio_pins)
        humidities = [-1.0] * len(self.gpio_pins)

        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = {executor.submit(self.read_sensor, pin): pin for pin in self.gpio_pins}
            for future in concurrent.futures.as_completed(futures):
                pin = futures[future]
                try:
                    temperature, humidity = future.result()
                    index = self.gpio_pins.index(pin)
                    temperatures[index] = temperature
                    humidities[index] = humidity

                    location = self.gpio_location[index]
                    if temperature != -1:
                        self.get_logger().info(f"Ubicación: {location}, GPIO {pin} -> Temp: {temperature:.1f}°C, Hum: {humidity:.1f}%")
                    else:
                        self.get_logger().warning(f"Ubicación: {location}, GPIO {pin} -> Error de lectura.")
                except Exception as exc:
                    self.get_logger().error(f"GPIO {pin} generó una excepción: {exc}")

        # Reorder data based on gpio_location_sorted
        sorted_temperatures = []
        sorted_humidities = []
        for sorted_location in self.gpio_location_sorted:
            if sorted_location in self.gpio_location:
                index = self.gpio_location.index(sorted_location)
                sorted_temperatures.append(temperatures[index])
                sorted_humidities.append(humidities[index])
            else:
                sorted_temperatures.append(-1.0)
                sorted_humidities.append(-1.0)

        # Publish data
        all_sensors_msg = DHTSensorData()
        all_sensors_msg.locations = self.gpio_location_sorted
        all_sensors_msg.temperatures = sorted_temperatures
        all_sensors_msg.humidities = sorted_humidities
        self.sensor_data_publisher_.publish(all_sensors_msg)


def main(args=None):
    rclpy.init(args=args)
    dht11_node = DHT11Node()
    try:
        rclpy.spin(dht11_node)
    except KeyboardInterrupt:
        dht11_node.get_logger().info('Nodo DHT11 detenido manualmente.')
    finally:
        dht11_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
