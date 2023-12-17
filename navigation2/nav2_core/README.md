# Nav2 Core

This package hosts the abstract interface (virtual base classes) for plugins to be used with the following:
- global planner (e.g., `nav2_navfn_planner`)
- controller (e.g., path execution controller, e.g `nav2_dwb_controller`)
- smoother (e.g., `nav2_ceres_costaware_smoother`)
- goal checker (e.g. `simple_goal_checker`)
- behaviors (e.g. `drive_on_heading`)
- progress checker (e.g. `simple_progress_checker`)
- waypoint task executor (e.g. `take_pictures`)
- exceptions in planning and control

The purposes of these plugin interfaces are to create a separation of concern from the system software engineers and the researcher / algorithm designers. Each plugin type is hosted in a "task server" (e.g. planner, recovery, control servers) which handles requests and multiple algorithm plugin instances. The plugins are used to compute a value back to the server without having to worry about ROS 2 actions, topics, or other software utilities. A plugin designer can simply use the tools provided in the API to do their work, or create new ones if they like internally to gain additional information or capabilities.

-------------------------------------------------

# Nav2 核心

该包托管插件的抽象接口（虚拟基类），以便与以下内容一起使用：
- 全局规划器（例如`nav2_navfn_planner`）
- 控制器（例如路径执行控制器，例如`nav2_dwb_controller`）
- 更平滑（例如`nav2_ceres_costaware_smoother`）
- 目标检查器（例如`simple_goal_checker`）
- 行为（例如“drive_on_heading”）
- 进度检查器（例如`simple_progress_checker`）
- 航点任务执行器（例如`take_pictures`）
- 规划和控制中的例外情况

这些插件接口的目的是使系统软件工程师和研究人员/算法设计者的关注点分离。 每个插件类型都托管在一个“任务服务器”（例如规划器、恢复、控制服务器）中，该服务器处理请求和多个算法插件实例。 这些插件用于计算返回服务器的值，而无需担心 ROS 2 操作、主题或其他软件实用程序。 插件设计者可以简单地使用 API 中提供的工具来完成他们的工作，或者如果他们愿意在内部获得额外的信息或功能，则可以创建新的工具。
