import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	# 创建一个LaunchDescription对象
	ld = LaunchDescription()

	robot_model_launch_dir = os.path.join(get_package_share_directory('robot_description'),'launch')
	cartographer_launch_dir = os.path.join(get_package_share_directory('cartographer_ros'), 'launch')

	# 定义要嵌套的launch文件的路径
	included_robot_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(robot_model_launch_dir, 'load_robot_model_into_gazebo.launch.py')))

	included_cartographer_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(cartographer_launch_dir, 'carto_2d.launch.py')))
	
	# 添加嵌套的launch文件到主LaunchDescription中
	ld.add_action(included_robot_launch)
	ld.add_action(included_cartographer_launch)

	# 添加要启动的节点
	include_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2'
	)	#（启动rviz2）

	ld.add_action(include_node)

	return ld

	