# nav2_bringup

The `nav2_bringup` package is an example bringup system for Nav2 applications. 

This is a very flexible example for nav2 bringup that can be modified for different maps/robots/hardware/worlds/etc. It is our expectation for an application specific robot system that you're mirroring `nav2_bringup` package and modifying it for your specific maps/robots/bringup needs. This is an applied and working demonstration for the default system bringup with many options that can be easily modified. 

Usual robot stacks will have a `<robot_name>_nav` package with config/bringup files and this is that for the general case to base a specific robot system off of.

Dynamically composed bringup (based on  [ROS2 Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html)) is optional for users. It can be used to compose all Nav2 nodes in a single process instead of launching these nodes separately, which is useful for embedded systems users that need to make optimizations due to harsh resource constraints. Dynamically composed bringup is used by default, but can be disabled by using the launch argument `use_composition:=False`.

* Some discussions about performance improvement of composed bringup could be found here: https://discourse.ros.org/t/nav2-composition/22175.

To use, please see the Nav2 [Getting Started Page](https://navigation.ros.org/getting_started/index.html) on our documentation website. Additional [tutorials will help you](https://navigation.ros.org/tutorials/index.html) go from an initial setup in simulation to testing on a hardware robot, using SLAM, and more. 

Note:
* gazebo should be started with both libgazebo_ros_init.so and libgazebo_ros_factory.so to work correctly.
* spawn_entity node could not remap /tf and /tf_static to tf and tf_static in the launch file yet, used only for multi-robot situations. Instead it should be done as remapping argument <remapping>/tf:=tf</remapping>  <remapping>/tf_static:=tf_static</remapping> under ros2 tag in each plugin which publishs transforms in the SDF file. It is essential to differentiate the tf's of the different robot.

-------------------------------------------
# nav2_bringup

`nav2_bringup` 包是 Nav2 应用程序的启动系统示例。

这是一个非常灵活的 nav2 启动示例，可以针对不同的地图/机器人/硬件/世界/等进行修改。 我们对特定于应用程序的机器人系统的期望是，您可以镜像“nav2_bringup”包并根据您的特定地图/机器人/bringup 需求对其进行修改。 这是默认系统启动的应用和工作演示，其中包含许多可以轻松修改的选项。

通常的机器人堆栈将有一个包含配置/启动文件的“<robot_name>_nav”包，这是基于特定机器人系统的一般情况。

动态组合的启动（基于 [ROS2 Composition](https://docs.ros.org/en/gactic/Tutorials/Composition.html)）对于用户来说是可选的。 它可用于在单个进程中组合所有 Nav2 节点，而不是单独启动这些节点，这对于需要进行优化的嵌入式系统用户非常有用
