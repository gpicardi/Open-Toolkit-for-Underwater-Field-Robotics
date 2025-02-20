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
    #file_subpath = 'description/robot.urdf.xacro'
    file_subpath = 'description/silver.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Use config file for rviz
    base_path = os.path.realpath(get_package_share_directory('silver')) # also tried without realpath
    rviz_path=base_path+'/config/rviz_config_file.rviz'

    # Configure nodes
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','rsp_silver.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    joint_state_publisher_gui_spawner = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(rviz_path)],
    )

    # Run the node
    return LaunchDescription([
        rsp,
        joint_state_publisher_gui_spawner,
        rviz_node
    ])


