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

    configuration_file = LaunchConfiguration('configuration_file', default='launch_dir/param/navigation2.yaml')
    
    return LaunchConfiguration([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[{'yaml_filename': configuration_file}]
        ),
        Node(
            package='nav2_smoother',
            executable='controller_server',
            output='screen',
            parameters=[{'yaml_filename': configuration_file}]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'yaml_filename': configuration_file}]  
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[{'yaml_filename': configuration_file}]    
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'yaml_filename': configuration_file}]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[{'yaml_filename': configuration_file}]
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[{'yaml_filename': configuration_file}]
        )
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        # )
    ])