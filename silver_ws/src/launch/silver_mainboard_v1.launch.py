import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'silver'
    file_subpath = 'description/silver.xacro'    #file_subpath = 'description/robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Specify controllers configurations
    controller_config = os.path.join(
        get_package_share_directory(
            pkg_name), "config", "silver_controllers.yaml" #"my_controllers.yaml"
    )

    # Include Robot State Publisher Launch
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_raw}, controller_config],
            output="screen"
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--controller-manager", "/controller_manager", "--inactive"],
        output="screen"
    )

    joint_trajectory_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager", "--inactive"],
        output="screen"
    )

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

    torches_spawner = Node(
        package="torches",
        executable="torches_control",
        output="screen"
    )

    # Run the node
    return LaunchDescription([
        controller_manager,
        rsp,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        forward_velocity_controller_spawner,
        joint_trajectory_position_controller_spawner,
        bno055_spawner,
        dht11_spawner,
        ltc2945_spawner,
        ms5837_spawner,
        ms8607_i2c0_spawner,
        #ms8607_i2c1_spawner,
        torches_spawner

    ])


