# Nav2 Controller

The Nav2 Controller is a Task Server in Nav2 that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with multiple plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package. It also contains progress checkers and goal checker plugins to abstract out that logic from specific controller implementations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available controller plugins. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-controller-server.html) for additional parameter descriptions and a [tutorial about writing controller plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html).

-------------------------------------------------

# Nav2 控制器

Nav2 控制器是 Nav2 中的一个任务服务器，它实现了 `nav2_msgs::action::FollowPath` 操作服务器。

实现“nav2_msgs::action::FollowPath”动作服务器的执行模块负责根据“nav2_planner”中规划器模块的计算路径生成机器人的命令速度。 nav2_controller 包被设计为加载多个用于路径执行的插件。 插件需要实现“nav2_core”包中“controller”头文件中定义的虚拟基类中的函数。 它还包含进度检查器和目标检查器插件，以从特定控制器实现中抽象出该逻辑。

有关当前已知且可用的控制器插件的列表，请参阅[导航插件列表](https://navigation.ros.org/plugins/index.html)。

请参阅其[配置指南页面](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)以获取其他参数说明和[有关编写控制器插件的教程](https://navigation.html)。 ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html）。
