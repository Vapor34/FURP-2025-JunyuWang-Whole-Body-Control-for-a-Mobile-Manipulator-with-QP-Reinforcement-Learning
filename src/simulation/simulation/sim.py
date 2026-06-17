import os
import time
import threading
import numpy as np
from pathlib import Path

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class MujocoRosBridge(Node):
    def __init__(self, model, data):
        super().__init__('mujoco_ur5e_bridge')
        self.model = model
        self.data = data

        self.joint_names = ARM_JOINT_NAMES
        self.joint_qpos_addr = []
        self.joint_qvel_addr = []
        
        #As a publisher, publish joint states as topic '/cur_joint_states'
        #message type:"sensor_msgs/JointState", topic name: "/cur_joint_states", queue size: 10
        self.joint_pub = self.create_publisher(JointState, '/cur_joint_states', 10)
        
        
        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name
            )

            if joint_id < 0:
                raise RuntimeError(f"Joint not found in MuJoCo model: {joint_name}")

            self.joint_qpos_addr.append(self.data.qpos[joint_id])
            self.joint_qvel_addr.append(self.data.qvel[joint_id])

        self.joint_sub = self.create_subscription(JointState, '/next_joint_states', self.receive_states, 10)

    def receive_states(self, msg):
        # Update the joint positions and velocities in the MuJoCo simulation based on the received message
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.joint_names:
                joint_id = self.joint_names.index(name)
                self.data.qpos[joint_id] = pos
                self.data.qvel[joint_id] = vel

        # create a timer to publish joint states every 0.02 seconds (50 Hz)
        self.timer = self.create_timer(0.02, self.publish_states)

    def publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        msg.position = [
            float(self.data.qpos[qpos_addr])
            for qpos_addr in self.joint_qpos_addr
        ]

        msg.velocity = [
            float(self.data.qvel[qvel_addr])
            for qvel_addr in self.joint_qvel_addr
        ]

        #publish message to topic '/joint_states'
        self.joint_pub.publish(msg)
        self.get_logger().info(f"Joint name: {msg.name}, position: {msg.position}, velocity: {msg.velocity}")

def ros_spin_thread(node):
    rclpy.spin(node)

def get_model_path():
    src_dir = Path(__file__).resolve().parents[2]
    return src_dir / "description" / "assets" / "mujoco" / "ur5e.xml"

def main():
    rclpy.init()

    model_path = get_model_path()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # 3. 启动 ROS 桥接节点（放在后台线程，防止阻塞 MuJoCo 的可视化窗口）
    ros_node = MujocoRosBridge(model, data)
    ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    ros_thread.start()

    print("MuJoCo simulation and ROS 2 Bridge started.")
    mujoco.viewer.launch(model, data)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
