# Nav2 Costmap_2d

The costmap_2d package is responsible for building a 2D costmap of the environment, consisting of several "layers" of data about the environment. It can be initialized via the map server or a local rolling window and updates the layers by taking observations from sensors. A plugin interface allows for the layers to be combined into the costmap and finally inflated via an inflation radius based on the robot footprint. The nav2 version of the costmap_2d package is mostly a direct ROS2 port of the ROS1 navigation stack version, with minimal noteable changes necessary due to support in ROS2. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-costmaps.html) for additional parameter descriptions for the costmap and its included plugins. The [tutorials](https://navigation.ros.org/tutorials/index.html) and [first-time setup guides](https://navigation.ros.org/setup_guides/index.html) also provide helpful context for working with the costmap 2D package and its layers. A [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html) is also provided to explain how to create costmap plugins.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

## To visualize the voxels in RVIZ:
- Make sure `publish_voxel_map` in `voxel_layer` param's scope is set to `True`.
- Open a new terminal and run:
  ```ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker```
    Here you can change `my_marker` to any topic name you like for the markers to be published on.

- Then add `my_marker` to RVIZ using the GUI.


### Errata:
- To see the markers in 3D, you will need to change the _view_ in RVIZ to a 3 dimensional view (e.g. orbit) from the RVIZ GUI.
- Currently due to some bug in rviz, you need to set the `fixed_frame` in the rviz display, to `odom` frame.
- Using pointcloud data from a saved bag file while using gazebo simulation can be troublesome due to the clock time skipping to an earlier time.

## Costmap Filters

### Overview

Costmap Filters - is a costmap layer-based instrument which provides an ability to apply to map spatial-dependent raster features named as filter-masks. These features are used in plugin algorithms when filling costmaps in order to allow robots to change their trajectory, behavior or speed when a robot enters/leaves an area marked in a filter masks. Examples of costmap filters include keep-out/safety zones where robots will never enter, speed restriction areas, preferred lanes for robots moving in industries and warehouses. More information about design, architecture of the feature and how it works could be found on Nav2 website: https://navigation.ros.org.

-------------------------------------------------

# Nav2 Costmap_2d

costmap_2d 包负责构建环境的 2D 成本图，由有关环境的多个“层”数据组成。 它可以通过地图服务器或本地滚动窗口进行初始化，并通过从传感器获取观察结果来更新图层。 插件接口允许将各层组合到成本图中，并最终通过基于机器人足迹的膨胀半径进行膨胀。 costmap_2d 包的 nav2 版本主要是 ROS1 导航堆栈版本的直接 ROS2 端口，由于 ROS2 的支持，需要进行最小的显着更改。

有关costmap及其包含的插件的其他参数说明，请参阅其[配置指南页面](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)。 [教程](https://navigation.ros.org/tutorials/index.html) 和[首次设置指南](https://navigation.ros.org/setup_guides/index.html) 还提供了有用的上下文 用于使用 costmap 2D 包及其图层。 还提供了一个[教程](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html)来解释如何创建costmap插件。

有关当前已知且可用的规划器插件的列表，请参阅[导航插件列表](https://navigation.ros.org/plugins/index.html)。

## 可视化 RVIZ 中的体素：
- 确保“voxel_layer”参数范围中的“publish_voxel_map”设置为“True”。
- 打开一个新终端并运行：
   ```ros2 运行 nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid Visualization_marker:=/my_marker```
     在这里，您可以将“my_marker”更改为您想要发布标记的任何主题名称。

- 然后使用 GUI 将 `my_marker` 添加到 RVIZ。


### 勘误表：
- 要以 3D 形式查看标记，您需要将 RVIZ 中的_视图_从 RVIZ GUI 更改为 3 维视图（例如轨道）。
- 目前由于 rviz 中的一些错误，您需要将 rviz 显示中的 `fixed_frame` 设置为 `odom` 框架。
- 使用凉亭模拟时使用保存的包文件中的点云数据可能会很麻烦，因为时钟时间会跳到较早的时间。

## 成本图过滤器

＃＃＃ 概述

Costmap Filters - 是一种基于 Costmap 图层的工具，它提供了应用于地图空间相关栅格要素（称为过滤器蒙版）的功能。 这些功能在填充成本图时在插件算法中使用，以便允许机器人在机器人进入/离开过滤器掩模中标记的区域时改变其轨迹、行为或速度。 成本图过滤器的示例包括机器人永远不会进入的禁区/安全区、速度限制区、工业和仓库中移动的机器人的首选车道。 有关设计、功能架构及其工作原理的更多信息，请访问 Nav2 网站：https://navigation.ros.org。
