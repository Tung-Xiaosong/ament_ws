# BT Navigator

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose and NavigateThroughPoses task interfaces. It is a [Behavior Tree](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-bt-navigator.html) for additional parameter descriptions, as well as the [Nav2 Behavior Tree Explanation](https://navigation.ros.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package.

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination(s). To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT. The BT XML can also be specified on a per-task basis so that your robot may have many different types of navigation or autonomy behaviors on a per-task basis.

-------------------------------------------------

# BT Navigator

BT Navigator（行为树导航器）模块实现了 NavigateToPose 和 NavigateThroughPoses 任务接口。 它是一个基于行为树的导航实现，旨在允许导航任务的灵活性并提供 一种轻松指定复杂机器人行为的方法。

其他参数说明请参见其[配置指南页面](https://navigation.ros.org/configuration/packages/configuring-bt-navigator.html)，以及[Nav2行为树说明](https:// navigation.ros.org/behavior_trees/index.html）页面解释了有关默认行为树的更多上下文以及此包中提供的示例。

＃＃ 概述

BT 导航器接收目标姿势并将机器人导航至指定目的地。 为此，该模块从文件中读取行为树的 XML 描述（由 Node 参数指定），并将其传递给通用 [BehaviorTreeEngine 类](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp)，该类 使用[Behavior-Tree.CPP库](https://github.com/BehaviorTree/BehaviorTree.CPP)动态创建和执行BT。 BT XML 还可以在每个任务的基础上指定，以便您的机器人可以在每个任务的基础上具有许多不同类型的导航或自主行为。
