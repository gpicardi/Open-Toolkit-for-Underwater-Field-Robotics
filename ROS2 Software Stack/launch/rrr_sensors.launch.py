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

    dht11_spawner = Node(
        package="dht11",
        executable="dht11",
        output="screen"
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

    ms8607_i2c1_spawner = Node(
        package="ms8607",
        executable="ms8607_i2c1",
        #executable="ms8607",
        output="screen"
    )

    # Run the node
    return LaunchDescription([
        bno055_spawner,
        dht11_spawner,
        ltc2945_spawner,
        ms5837_spawner,
        ms8607_i2c0_spawner,
        #ms8607_i2c1_spawner
    ])


