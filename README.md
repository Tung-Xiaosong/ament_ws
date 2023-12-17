# ament_ws
My ROS2 workspace

---------------------------------------------------
# 创建工作空间
mkdir -p catkin_ws/src

# 创建功能包
ros2 pkg create --build-type ament_cmake YOUR_PAK_NAME --dependencies rclcpp std_msgs

ros2 pkg create --build-type ament_python YOUR_PAK_NAME --dependencies rclpy std_msgs

# 编译
* colcon编译器和ROS2之间是相互独立的，所以安装ROS2不会自动将colcon作为依赖安装。
sudo apt install python3-colcon-common-extensions

colcon build

编译工作空间中的一个功能包
colcon build --packages-select YOUR_PKG_NAME

# 查看urdf模型结构	
urdf_to_graphviz mbot_base.urdf //在模型文件夹下，生成pdf

# 安装gazebo
sudo apt install ros-humble-gazebo-*
ros2 launch gazebo_ros gazebo.launch.py

# 安装xacro
sudo apt install ros-humble-xacro

sudo apt install rospack-tools

# ROS2 
ros2 pkg prefix  <package-name>输出某个包所在路径的前缀

ros2 pkg executables <pkg>列出功能包的可执行文件 ros2 pkg executables

ros2 pkg xml <package-name>每一个功能包都有一个标配的manifest.xml文件，用于记录这个包的名字，构建工具，编译信息，拥有者，干啥用的等信息。通过这个信息，就可以自动为该功能包安装依赖，构建时确定编译顺序等

git clone ... -b humble

colcon build --symlink-install

ros2 topic list -t查看话题消息类型

ros2 interface proto sensor_msgs/msg/Image输出某一个接口所有属性

yuxiangros40 c++ P53

ros2 service find example_interfaces/srv/AddTwoInts查找使用某一接口的服务

C++用成员函数作回调函数，用std::bind()
回调组

colcon build --parallel-workers 6

sudo apt install ros-humble-rqt-tf-tree

ros2 run tf2_ros tf2_echo B P

tf2_monitor查看所有的发布者和频率。

rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

