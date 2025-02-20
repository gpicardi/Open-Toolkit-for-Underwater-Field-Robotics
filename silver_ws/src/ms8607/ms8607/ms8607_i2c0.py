import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from smbus2 import SMBus
from time import sleep

#Define sensor address, register offsets and number of bytes
MS8607_ADDR = 0x76 #(118)
MS8607_ADDR_HUMIDITY = 0x40 #(64)
PRESSURE_OFF = 0
PRESSURE_NBYTES = 2
PRESSURE_RES = 1.0
TEMPERATURE_OFF = 0
TEMPERATURE_NBYTES = 2
TEMPERATURE_RES = 1.0
HUMIDITY_OFF = 0
HUMIDITY_NBYTES = 2
HUMIDITY_RES = 1.0
RESET_OFF = 0x1E #(30)

class Ms8607Node(Node):
    
    def __init__(self, bus=0):        
        try:
            self._bus = SMBus(bus)
        except:
            print("Bus %d is not available."%bus)
            print("Available busses are listed as /dev/i2c*")
            self._bus = None
        
        #Reset command
        self._bus.write_byte(MS8607_ADDR, RESET_OFF)
        sleep(0.5)

        # Calibration
        # Read 12 bytes of calibration data
        # Pressure sensitivity | SENST1
        data1 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA2, 2)
        # Pressure offset | OFFT1
        data2 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA4, 2)
        # Temperature coefficient of pressure sensitivity | TCS
        data3 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA6, 2)
        # Temperature coefficient of pressure offset | TCO
        data4 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xA8, 2)
        # Reference temperature | TREF
        data5 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xAA, 2)
        # Temperature coefficient of the temperature | TEMPSENS
        data6 = self._bus.read_i2c_block_data(MS8607_ADDR, 0xAC, 2)

        # Convert the data
        self.c1 = data1[0] * 256 + data1[1]
        self.c2 = data2[0] * 256 + data2[1]
        self.c3 = data3[0] * 256 + data3[1]
        self.c4 = data4[0] * 256 + data4[1]
        self.c5 = data5[0] * 256 + data5[1]
        self.c6 = data6[0] * 256 + data6[1]

        # Initialize parent (ROS Node)
        super().__init__('ms8607_i2c0')
        self.pressure_publisher_ = self.create_publisher(Float64, '/ms8607_i2c0/pressure', 10)
        self.temperature_publisher_ = self.create_publisher(Float64, '/ms8607_i2c0/temperature', 10)
        self.humidity_publisher_ = self.create_publisher(Float64, '/ms8607_i2c0/humidity', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.get_logger().info("Sensor MS8607 on bus i2c-0 correctly initialized.")

    def timer_callback(self):
        try:
            pressure_msg = Float64()
            temperature_msg = Float64()
            pressure_msg.data, temperature_msg.data = self.get_pressure_and_temperature()
            
            humidity_msg = Float64()
            humidity_msg.data = self.get_humidity()
            
            self.pressure_publisher_.publish(pressure_msg)
            self.temperature_publisher_.publish(temperature_msg)
            self.humidity_publisher_.publish(humidity_msg)

            self.i +=1
        except:
            self.i +=1
        
        """self.get_logger().info("Publishing Pressure: %.2f mbar" %pressure_msg.data)
        self.get_logger().info("Publishing Temperature: %.2f C" %temperature_msg.data)
        self.get_logger().info("Publishing Humidity: %.2f %%" %humidity_msg.data)"""

    def get_data(self, addr, off, nbytes):
        data = self._bus.read_i2c_block_data(addr, off, nbytes)
        return data
    
    def get_pressure_and_temperature(self): #Change address with constants
        # MS8607_02BA address, 0x76(118)
        #		0x40(64)	Initiate pressure conversion(OSR = 256)
        self._bus.write_byte(MS8607_ADDR, 0x40)

        sleep(0.5)

        # Read data back from 0x00(0), 3 bytes, D1 MSB2, D1 MSB1, D1 LSB
        # Digital pressure value
        data = self._bus.read_i2c_block_data(MS8607_ADDR, 0x00, 3)

        D1 = data[0] * 65536 + data[1] * 256 + data[2]

        # MS8607_02BA address, 0x76(118)
        #		0x50(64)	Initiate temperature conversion(OSR = 256)
        self._bus.write_byte(MS8607_ADDR, 0x50)
        sleep(0.5)

        # Read data back from 0x00(0), 3 bytes, D2 MSB2, D2 MSB1, D2 LSB
        # Digital temperature value
        data0 = self._bus.read_i2c_block_data(MS8607_ADDR, 0x00, 3)

        # Convert the data
        D2 = data0[0] * 65536 + data0[1] * 256 + data0[2]
        dT = D2 - self.c5 * 256
        Temp = 2000 + dT * self.c6 / 8388608
        OFF = self.c2 * 131072 + (self.c4 * dT) / 64
        SENS = self.c1 * 65536 + (self.c3 * dT ) / 128

        if Temp >= 2000 :
            Ti = 5 * (dT * dT) / 274877906944
            OFFi = 0
            SENSi= 0
        elif Temp < 2000 :
            Ti = 3 * (dT * dT) / 8589934592
            OFFi= 61 * ((Temp - 2000) * (Temp - 2000)) / 16
            SENSi= 29 * ((Temp - 2000) * (Temp - 2000)) / 16
            if Temp < -1500:
                OFFi = OFFi + 17 * ((Temp + 1500) * (Temp + 1500))
                SENSi = SENSi + 9 * ((Temp + 1500) * (Temp +1500))
        OFF2 = OFF - OFFi
        SENS2= SENS - SENSi
        cTemp = (Temp - Ti) / 100.0
        pressure = ((((D1 * SENS2) / 2097152) - OFF2) / 32768.0) / 100.0
        return pressure, cTemp
    
    def get_humidity(self): 
        # MS8607_02BA address, 0x40(64)
        #		0xFE(254)	Send reset command
        self._bus.write_byte(MS8607_ADDR_HUMIDITY, 0xFE)
        sleep(0.3)

        #		0xF5(245)	Send NO Hold master command
        self._bus.write_byte(MS8607_ADDR_HUMIDITY, 0xF5)
        sleep(0.5)

        # Read data back from device
        data0 = self._bus.read_byte(MS8607_ADDR_HUMIDITY)
        data1 = 0

        # Convert the data
        D3 = data0 * 256 + data1

        humidity = (-6.0 + (125.0 * (D3 / 65536.0)))
        return humidity

def main(args=None):
    rclpy.init(args=args)
    ms8607_i2c0_publisher = Ms8607Node()
    rclpy.spin(ms8607_i2c0_publisher)

    ms8607_i2c0_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()