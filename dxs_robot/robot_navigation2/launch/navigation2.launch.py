import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_navigation2')

    controller_configuration_file = LaunchConfiguration('controller_configuration_file', default='launch_dir/param/controller.yaml')
    
    return LaunchConfiguration([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[{'yaml_filename': controller_configuration_file}]
        )
        Node(
            package='nav2_smoother',
            executable='controller_server',
            output='screen',
            parameters=[{'yaml_filename': controller_configuration_file}]
        )
    ])