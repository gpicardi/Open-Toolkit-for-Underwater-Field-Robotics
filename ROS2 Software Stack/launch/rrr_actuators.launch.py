import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'rrr_manipulator'
    file_subpath = 'description/rrr_manipulator.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    # Specify controllers configurations
    controller_config = os.path.join(
        get_package_share_directory(
            pkg_name), "config", "rrr_controllers.yaml" 
    )

    # Include Robot State Publisher Launch (RSP)

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    # Configure the node
    params={'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params] # add other parameters here if required
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

    # Run the node
    return LaunchDescription([
        controller_manager,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        joint_trajectory_position_controller_spawner,
        forward_position_controller_spawner,
        forward_velocity_controller_spawner
    ])


