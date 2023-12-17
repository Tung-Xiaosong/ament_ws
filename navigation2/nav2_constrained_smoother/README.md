# Constrained Smoother

A smoother plugin for `nav2_smoother` based on the original deprecated smoother in `nav2_smac_planner` by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) and put into operational state by [**RoboTech Vision**](https://robotechvision.com/). Suitable for applications which need planned global path to be pushed away from obstacles and/or for Reeds-Shepp motion models.

See documentation on navigation.ros.org: https://navigation.ros.org/configuration/packages/configuring-constrained-smoother.html


Example of configuration (see indoor_navigation package of this repo for a full launch configuration):

```
smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["SmoothPath"]

    SmoothPath:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"
      reversing_enabled: true       # whether to detect forward/reverse direction and cusps. Should be set to false for paths without orientations assigned
      path_downsampling_factor: 3   # every n-th node of the path is taken. Useful for speed-up
      path_upsampling_factor: 1     # 0 - path remains downsampled, 1 - path is upsampled back to original granularity using cubic bezier, 2... - more upsampling
      keep_start_orientation: true  # whether to prevent the start orientation from being smoothed
      keep_goal_orientation: true   # whether to prevent the gpal orientation from being smoothed
      minimum_turning_radius: 0.40  # minimum turning radius the robot can perform. Can be set to 0.0 (or w_curve can be set to 0.0 with the same effect) for diff-drive/holonomic robots
      w_curve: 30.0                 # weight to enforce minimum_turning_radius
      w_dist: 0.0                   # weight to bind path to original as optional replacement for cost weight
      w_smooth: 2000000.0           # weight to maximize smoothness of path
      w_cost: 0.015                 # weight to steer robot away from collision and cost

      # Parameters used to improve obstacle avoidance near cusps (forward/reverse movement changes)
      # See the [docs page](https://navigation.ros.org/configuration/packages/configuring-constrained-smoother) for further clarification
      w_cost_cusp_multiplier: 3.0   # option to have higher weight during forward/reverse direction change which is often accompanied with dangerous rotations
      cusp_zone_length: 2.5         # length of the section around cusp in which nodes use w_cost_cusp_multiplier (w_cost rises gradually inside the zone towards the cusp point, whose costmap weight equals w_cost*w_cost_cusp_multiplier)

      # Points in robot frame to grab costmap values from. Format: [x1, y1, weight1, x2, y2, weight2, ...]
      # IMPORTANT: Requires much higher number of iterations to actually improve the path. Uncomment only if you really need it (highly elongated/asymmetric robots)
      # See the [docs page](https://navigation.ros.org/configuration/packages/configuring-constrained-smoother) for further clarification
      # cost_check_points: [-0.185, 0.0, 1.0]

      optimizer:
        max_iterations: 70            # max iterations of smoother
        debug_optimizer: false        # print debug info
        gradient_tol: 5e3
        fn_tol: 1.0e-15
        param_tol: 1.0e-20
```

Note: Smoothing paths which contain multiple subsequent poses at one point (e.g. in-place rotations from Smac lattice planners) is currently not supported

Note: Constrained Smoother is recommended to be used on a path with a bounded length. TruncatePathLocal BT Node can be used for extracting a relevant path section around robot (in combination with DistanceController to achieve periodicity)

-------------------------------------------------

# 约束平滑器

`nav2_smoother` 的平滑插件基于 [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) 的 `nav2_smac_planner` 中原始已弃用的平滑器，并由 [* 置于运行状态 *RoboTech Vision**](https://robotechvision.com/)。 适用于需要将规划的全局路径推离障碍物和/或 Reeds-Shepp 运动模型的应用。

请参阅 navigation.ros.org 上的文档：https://navigation.ros.org/configuration/packages/configuring-constrained-smoother.html


配置示例（有关完整的启动配置，请参阅此存储库的 Interior_navigation 包）：

````
更平滑的服务器：
   ros__参数：
     use_sim_time：真
     smoother_plugins：[“SmoothPath”]

     平滑路径：
       插件：“nav2_constrained_smoother/ConstrainedSmoother”
       reversing_enabled: true # 是否检测正向/反向和尖点。 对于没有分配方向的路径应设置为 false
       path_downsampling_factor: 3 # 路径中的每个第 n 个节点都会被采样。 对于加速很有用
       path_upsampling_factor: 1 # 0 - 路径保持下采样，1 - 使用三次贝塞尔曲线将路径上采样回原始粒度，2... - 更多上采样
       keep_start_orientation: true # 是否防止起始方向被平滑
       keep_goal_orientation: true # 是否防止gpal方向被平滑
       minimum_turning_radius: 0.40 # 机器人可以执行的最小转弯半径。 对于 diff-drive/holonomic 机器人可以设置为 0.0（或者 w_curve 可以设置为 0.0 具有相同的效果）
       w_curve: 30.0 # 强制执行minimum_turning_radius的权重
       w_dist: 0.0 # 将路径绑定到原始路径的权重，作为成本权重的可选替代
       w_smooth: 2000000.0 # 最大化路径平滑度的权重
       w_cost: 0.015 # 引导机器人远离碰撞的重量和成本

       # 用于改善尖点附近避障的参数（向前/向后移动变化）
       # 有关进一步说明，请参阅[文档页面](https://navigation.ros.org/configuration/packages/configuring-constrained-smoother)
       w_cost_cusp_multiplier: 3.0 # 在前进/后退方向改变期间具有更高权重的选项，这通常伴随着危险的旋转
       cusp_zone_length: 2.5 # 尖点周围节点使用w_cost_cusp_multiplier的部分的长度（w_cost在区域内向尖点逐渐上升，其costmap权重等于w_cost*w_cost_cusp_multiplier）

       # 机器人框架中用于获取成本图值的点。 格式：[x1, y1, 权重1, x2, y2, 权重2, ...]
       # 重要提示：需要更多的迭代次数才能真正改进路径。 仅当您确实需要时才取消注释（高度拉长/不对称机器人）
       # 有关进一步说明，请参阅[文档页面](https://navigation.ros.org/configuration/packages/configuring-constrained-smoother)
       # cost_check_points: [-0.185, 0.0, 1.0]

       优化器：
         max_iterations: 70 # 平滑器的最大迭代次数
         debug_optimizer: false # 打印调试信息
         梯度托尔：5e3
         fn_tol：1.0e-15
         参数tol：1.0e-20
````

注意：当前不支持在一个点包含多个后续姿势的平滑路径（例如，来自 Smac 晶格规划器的就地旋转）

注意：建议在具有有限长度的路径上使用约束平滑器。 TruncatePathLocal BT Node可用于提取机器人周围的相关路径段（与DistanceController结合实现周期性）
