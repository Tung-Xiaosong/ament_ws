#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2节点示例-发布“Hello World”日志信息, 使用面向过程的实现方式
"""

import rclpy                                     # ROS2 Python接口库 引入ros2 python库
from rclpy.node import Node                      # 引入 ROS2 节点类
import time

def main(args=None):                             # ROS2节点主入口main函数
    rclpy.init(args=args)                        # ROS2 Python接口初始化
    node = Node("node_helloworld")               # 创建ROS2节点对象并进行初始化
    
    while rclpy.ok():                            # ROS2系统是否正常运行
        node.get_logger().info("Hello World")    # ROS2日志输出 = printf
        time.sleep(0.5)                          # 休眠控制循环时间
    
    node.destroy_node()                          # 销毁节点对象    
    rclpy.shutdown()                             # 关闭ROS2 Python接口




#  编程接口初始化
#  创建节点并初始化
#  实现节点功能
#  销毁节点并关闭接口

#  编写或者修改程序后，一定要重新编译，编译过程会将src的代码拷贝到install中，ros2 run是install里的程序