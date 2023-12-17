from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	# 创建一个LaunchDescription对象
	ld = LaunchDescription()
	
	# 定义要嵌套的launch文件的路径
	included_launch_1 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource('/home/dxs/catkin_ws/src/ros2_21_tutorials/learning_gazebo/launch/load_robot_model_into_gazebo.launch.py')
	)	#（启动gazebo机器人模型仿真）
	
	included_launch_2 = IncludeLaunchDescription(
		PythonLaunchDescriptionSource('/home/dxs/catkin_ws/src/SLAM/slam_gmapping/slam_gmapping/launch/slam_gmapping.launch.py')
	)	#（启动gmapping建图）

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

	