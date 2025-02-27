import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'bringup'

    # Include RRR Actuator Launch
    rrr_actuators = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name), 'launch/rrr_actuators.launch.py'
                )])#, launch_arguments={'use_sim_time': 'false'}.items() # , 'use_ros2_control': 'true'
    )

    # Include RRR Sensor Launch
    rrr_sensors = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name), 'launch/rrr_sensors.launch.py'
                )]))

    # Include Torches Control Launch
    torches_spawner = Node(
        package="torches",
        executable="torches_control",
        output="screen"
    )

    # Run the node
    return LaunchDescription([
        rrr_actuators,
        rrr_sensors,
        torches_spawner
    ])


