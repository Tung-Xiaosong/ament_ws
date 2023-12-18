import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.condition import IfCondition
from launch.substitution import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    launch_dir = get_package_share_directory('robot_navigation2')

    # Load param file
    configuration_file = LaunchConfiguration('configuration_file', default='launch_dir/param/navigation2.yaml')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': configuration_file}]        
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'yaml_filename': configuration_file}]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node
    ])
    