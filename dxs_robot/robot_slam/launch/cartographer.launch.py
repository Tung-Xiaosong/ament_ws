from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	# 创建一个LaunchDescription对象
	ld = LaunchDescription()
	
	# 定义要嵌套的launch文件的路径
	included_launch_1 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource('/home/dxs/colcon_ws/src/dxs_robot/robot_description/launch/load_robot_model_into_gazebo.launch.py')
	)	#（启动gazebo机器人模型仿真）
	
	included_launch_2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource('/home/dxs/colcon_ws/src/SLAM/cartographer_ros/cartographer_ros/launch/carto_2d.launch.py')
	)	#（启动cartographer建图）

	# 添加嵌套的launch文件到主LaunchDescription中
	ld.add_action(included_launch_1)
	ld.add_action(included_launch_2)

	# 添加要启动的节点
	include_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2'
	)	#（启动rviz2）

	ld.add_action(include_node)

	return ld

	