# 启动机器人仿真模型和模拟世界
ros2 launch robot_description load_robot_model_into_gazebo.launch.py

# 启动建图
ros2 launch robot_slam gmapping.launch.py
ros2 launch robot_slam cartographer.launch.py

