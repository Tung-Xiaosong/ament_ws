# """
#   Copyright 2018 The Cartographer Authors
#   Copyright 2022 Wyca Robotics (for the ros2 conversion)

#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
# """

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration    #dxs add
from launch_ros.substitutions import FindPackageShare   #dxs add
from launch import actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # pkg_dir = get_package_share_directory('robot_description')
    # rviz_config_file= os.path.join(get_package_share_directory('my_slam'), 'rviz2', 'carto_slam.rviz')

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_dir, 'gazebo_lab_world.launch.py')
    #     ),
    # )

    # lua_configuration_directory = os.path.join(
    #     get_package_share_directory('my_slam'), 'lua')
    # lua_configuration_name = 'carto_2d.lua'

    # cartographer = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_node',
    #     name='cartographer_node',
    #     arguments=['-configuration_directory', lua_configuration_directory,
    #                '-configuration_basename', lua_configuration_name ],
    #     remappings = [('scan', 'scan')],
    #     output='screen',
    # )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'carto_2d.lua'],  #注意这里有一个.lua文件的映射，改为自己配置的.lua文件
        remappings = [
            ('echoes', 'horizontal_laser_2d')],
        output = 'screen'
        )

    cartographer_map = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',   #dxs change occupancy_grid_node
        #name='cartographer_occupancy_grid_node', #dxs change occupancy_grid_node
        arguments=['-resolution', '0.05'],
        output='screen',
    )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen',
    # )

    return LaunchDescription([
        #gazebo,
        cartographer_node,
        cartographer_map,
        #rviz,
    ])

# --------------------------------------------------------------------------------------------
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node, SetRemap
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import os

# def generate_launch_description():

#     ## ***** Launch arguments *****
#     # 是否使用仿真时间，真实的机器人不需要，设置为False
#     use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'true')#dxs change False

#     ## ***** File paths ******
#     # 找到cartographer功能包的地址
#     pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
#     urdf_dir = os.path.join(pkg_share, 'urdf')
#     urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
#     with open(urdf_file, 'r') as infp:
#         robot_desc = infp.read()

#     ## ***** Nodes *****
#     #=====================声明三个节点，cartographer/occupancy_grid_node/robot_state_publisher=====================
#     robot_state_publisher_node = Node(
#         package = 'robot_state_publisher',
#         executable = 'robot_state_publisher',
#         parameters=[
#             {'robot_description': robot_desc},
#             {'use_sim_time': LaunchConfiguration('use_sim_time')}],
#         output = 'screen'
#         )

#     cartographer_node = Node(
#         package = 'cartographer_ros',
#         executable = 'cartographer_node',
#         parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#         arguments = [
#             '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
#             '-configuration_basename', 'carto_2d.lua'],  #注意这里有一个.lua文件的映射，改为自己配置的.lua文件
#         remappings = [
#             ('echoes', 'horizontal_laser_2d')],
#         output = 'screen'
#         )

#     cartographer_occupancy_grid_node = Node(
#         package = 'cartographer_ros',
#         executable = 'cartographer_occupancy_grid_node',
#         parameters = [
#             {'use_sim_time': True},
#             {'resolution': 0.05}],
#         )

#     return LaunchDescription([
#         use_sim_time_arg,
#         # Nodes
#         robot_state_publisher_node,
#         cartographer_node,
#         cartographer_occupancy_grid_node,
#     ])
# --------------------------------------------------------------------------------------------------