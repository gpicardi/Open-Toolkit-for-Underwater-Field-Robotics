import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    bno055_config = os.path.join(
        get_package_share_directory('bno055'),
        'config',
        'bno055_params_i2c.yaml'
        )
        
    bno055_spawner=Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [bno055_config]
    )

    ltc2945_spawner = Node(
        package="ltc2945",
        executable="ltc2945",
        output="screen"
    )

    ms5837_spawner = Node(
        package="ms5837",
        executable="ms5837",
        output="screen"
    )

    ms8607_i2c0_spawner = Node(
        package="ms8607",
        executable="ms8607_i2c0",
        output="screen"
    )

    # Run the node
    gpio_pins = [4, 5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27] 
    sensor_locations = ['F3','T1','F1','F4','C5','F5','ControlC','CameraC','C3','T4','T3','T2','C4','T5','C2','T0','F2','C0','C1','F0']
    dht11_nodes = [Node(
            package='dht11',
            executable='dht11_s',
            name=f'dht11_sensor_node_{sensor_locations[i]}',
            parameters=[
                {'gpio_pin': gpio_pins[i], 'sensor_type': 'DHT11', 'sensor_location': sensor_locations[i]},
            ],
        ) for i in range(0, len(gpio_pins))]

    return LaunchDescription([
        bno055_spawner,
        ltc2945_spawner,
        ms5837_spawner,
        ms8607_i2c0_spawner,
        *dht11_nodes
    ])


