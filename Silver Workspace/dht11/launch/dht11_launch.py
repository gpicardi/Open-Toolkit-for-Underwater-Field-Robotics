# launch/dht11_sensor_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import board

def generate_launch_description():
    gpio_pins = [4, 5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27] 
    #gpio_pins = [board.D4, board.D5, board.D6, board.D7, board.D9, board.D10, board.D11, board.D13, board.D14, board.D15, board.D16, board.D17, board.D20, board.D21, board.D22, board.D23, board.D24, board.D25, board.D26, board.D27] # List of GPIO pins
    sensor_locations = ['F3','T1','F1','F4','C5','F5','ControlC','CameraC','C3','T4','T3','T2','C4','T5','C2','T0','F2','C0','C1','F0']

    return LaunchDescription([
        Node(
            package='dht11',
            executable='dht11_s',
            name=f'dht11_sensor_node_{sensor_locations[i]}',
            parameters=[
                {'gpio_pin': gpio_pins[i], 'sensor_type': 'DHT11', 'sensor_location': sensor_locations[i]},
            ],
        ) for i in range(0, len(gpio_pins))
    ])
