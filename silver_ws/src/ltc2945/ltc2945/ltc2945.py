import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from smbus2 import SMBus
from time import sleep

LTC2945_ADDR = 103
ADIN_OFF = 0x28
ADIN_NBYTES = 2
POWER_OFF = 0x5
POWER_NBYTES = 3
DELTA_SENSE_OFF = 0x14
DELTA_SENSE_NBYTES = 2
VIN_OFF = 0x1E
VIN_NBYTES = 2

#This must be verified, it depends on SHUNT resistor and it is connected to maximum values
DELTA_RES = 25E-3/3 #25E-3 is the resolution, 3 is the shunt resistor value in mOhm
VIN_RES = 0.025
ADIN_RES = 0.5E-3
POWER_RES = 6E-7

class Ltc2945Node(Node):
    
    def __init__(self, bus=0):        
        try:
            self._bus = SMBus(bus)
        except:
            print("Bus %d is not available."%bus)
            print("Available busses are listed as /dev/i2c*")
            self._bus = None

        # Initialize parent (ROS Node)
        super().__init__('ltc2945')
        self.vin_publisher_ = self.create_publisher(Float64, '/ltc2945/vin', 10)
        self.power_publisher_ = self.create_publisher(Float64, '/ltc2945/power', 10)
        self.current_publisher_ = self.create_publisher(Float64, '/ltc2945/current', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.get_logger().info("Sensor LTC2945 on bus i2c-0 correctly initialized.")

    def timer_callback(self):
        try:
            vin_msg = Float64()
            vin_msg.data = self.get_vin()
            power_msg = Float64()
            power_msg.data = self.get_power()
            current_msg = Float64()
            current_msg.data = self.get_delta_sense()
		
            self.vin_publisher_.publish(vin_msg)
            self.power_publisher_.publish(power_msg)
            self.current_publisher_.publish(current_msg)

            self.i +=1
        except:
            self.i +=1
        
        """self.get_logger().info("Publishing Vin: %.2f" %vin_msg.data)
        self.get_logger().info("Publishing Power: %.2f" %power_msg.data)
        self.get_logger().info("Publishing Current: %.7f" %current_msg.data)"""

    def get_data(self, addr, off, nbytes):
        data = self._bus.read_i2c_block_data(addr, off, nbytes)
        return data
    
    def get_adin(self):
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, ADIN_OFF, ADIN_NBYTES)
        adin = (data[0] << 4) | (data[1] >> 4)
        adin = adin * ADIN_RES
        return adin
    
    def get_delta_sense(self):
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, DELTA_SENSE_OFF, DELTA_SENSE_NBYTES)
        delta_sense = (data[0] << 4) | (data[1] >> 4)
        delta_sense = delta_sense*25e-3/3#DELTA_RES
        return delta_sense
    
    def get_power(self):
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, POWER_OFF, POWER_NBYTES)
        power = (data[0] + (data[1] << 8) + (data[2] << 16))
        power = power * POWER_RES
        return power
    
    def get_vin(self):
        data = self._bus.read_i2c_block_data(LTC2945_ADDR, VIN_OFF, VIN_NBYTES)
        vin = (data[0] << 4) | (data[1] >> 4)
        vin = vin * VIN_RES
        return vin

def main(args=None):
    rclpy.init(args=args)
    ltc2945_publisher = Ltc2945Node()
    rclpy.spin(ltc2945_publisher)

    ltc2945_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()