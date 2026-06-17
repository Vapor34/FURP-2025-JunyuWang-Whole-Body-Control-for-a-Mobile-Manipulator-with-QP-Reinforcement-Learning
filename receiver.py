import os
import time
import threading
import numpy as np

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        # 初始化节点，命名为 'joint_state_listener'
        super().__init__('receiver_node')
        
        # 创建订阅者
        # 参数1: 消息类型 (JointState)
        # 参数2: 话题名称 ('/joint_states')
        # 参数3: 回调函数 (self.listener_callback)
        # 参数4: QoS 队列长度 (10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # 防止未使用的变量警告

    def listener_callback(self, msg):
        """
        当接收到 /joint_states 话题的数据时，触发此回调函数
        """
        self.get_logger().info('--- 收到新的关节状态 ---')
        
        # 将 joint names, positions, 和 velocities 组合在一起遍历并打印
        # 使用 zip 函数可以方便地将这三个列表一一对应
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            # 格式化输出，保留四位小数，使终端显示更整洁
            self.get_logger().info(f"关节: {name:<12} | 位置 (rad): {pos:>7.4f} | 速度 (rad/s): {vel:>7.4f}")

def main(args=None):
    rclpy.init(args=args)

    # 实例化监听节点
    listener_node = JointStateListener()

    try:
        # 保持节点运行，持续监听
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 中断信号，优雅退出
        listener_node.get_logger().info('监听节点已停止。')
    finally:
        # 销毁节点并关闭 rclpy
        listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()