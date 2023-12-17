# nav2_amcl
自适应蒙特卡罗定位 (AMCL) 是一种概率定位模块，它使用 2D 激光扫描仪在给定的已知地图中估计机器人的位置和方向（即姿势）。这很大程度上是 ROS 1 的重构端口，没有任何算法更改。

# nav2_behaviors
实现了一个用于执行行为的任务服务器。

# nav2_behavior_tree

# nav2_bringup
用于启动导航系统，包括启动导航、路径规划和控制节点，以及一些参数和配置文件。

# nav2_bt_navigator

# nav2_collision_monitor

# nav2_common
包含一些与导航2相关的通用代码、数据结构和功能，它可以在不同的导航模块中共享。

# nav2_constrained_smoother

# nav2_controller

# nav2_core
包含导航2的核心接口和基础类，用于创建导航相关的插件和组件。

# nav2_costmap_2d
提供了2D代价地图的生成和管理功能，它用于将传感器数据融合到地图中，并生成用于路径规划的代价地图。

# nav2_dwb_controller
这是DiffDrive Welded Base（DWB）控制器的实现，用于执行底盘的控制。

# nav2_lifecycle_manager
提供了一种在导航系统生命周期内管理不同节点的方法，以便可以在不同阶段加载和卸载导航组件。

# nav2_map_server
提供地图数据，包括静态地图和语义地图，供导航系统使用。

# nav2_mppi_controller

# nav2_msgs
定义了导航系统中使用的消息类型。它包含了一些用于描述目标点、路径、状态等导航相关信息的消息定义。

# nav2_navfn_planner
使用 A* 或 Dijkstra 实现导航功能规划器。

# nav2_planner
功能包实现了全局路径规划算法。它提供了不同的路径规划器实现，如A*、Dijkstra等，用于生成机器人在环境中的全局路径。

# nav2_regulated_pure_pursuit_controller

# nav2_rotation_shim_controller

# nav2_rviz_plugins
包含了一些用于在RViz中可视化导航数据的插件。

# nav2_simple_commander

# nav2_smac_planner

# nav2_smoother

# nav2_system_tests

# nav2_theta_star_planner

# nav2_util
包含与导航2相关的一般实用程序和工具。

# nav2_velocity_smoother

# nav2_voxel_grid

# nav2_waypoint_follower
实现了一种用于跟踪路径上的路标点的控制器，可用于实现高级路径跟踪。

# navigation2

# tools

-------------------------------------------------

#==============控制器及其实现相关功能包======================#
nav2_controller　｜　控制器
nav2_dwb_controller | DWB控制器，Nav2控制器的一个实现
nav2_regulated_pure_pursuit_controller | 纯追踪控制器，Nav2控制器的一个实现

#==============规划器及其实现相关功能包======================#
nav2_planner | Nav2规划器
nav2_navfn_planner　｜　navfn规划器，Nav2规划器的一个实现
smac_planner | smac规划器，Nav2规划器的一个实现

#=====================恢复器==============================#
nav2_recoveries | Nav2恢复器

#=====================行为树节点及其定义====================#
nav2_bt_navigator |　导航行为树
nav2_behavior_tree | 行为树节点插件定义

#=====================地图和定位===========================#
nav2_map_server　｜　地图服务器
nav2_costmap_2d　｜　2D代价地图
nav2_voxel_grid | 体素栅格
nav2_amcl | 自适应蒙特卡洛定位。　　状态估计，输入地图、激光、里程计数据，输出机器人map和odom之间的位资关系。

#=====================通用插件系统管理等====================#
nav2_bringup | 启动入口
nav2_common　｜　公共功能包
nav2_msgs　｜　通信相关消息定义
nav2_util | 常用工具
nav2_lifecycle_manager |节点生命周期管理器　
nav2_rviz_plugins | RVIZ插件

#=====================核心定义============================#
nav2_core　｜　Nav2核心包
navigation2 | nav2导航汇总配置

#=====================应用================================#
nav2_waypoint_follower | 路点跟踪

#=====================测试=================================#
nav2_system_tests | 系统测试Copy to clipboardErrorCopied

