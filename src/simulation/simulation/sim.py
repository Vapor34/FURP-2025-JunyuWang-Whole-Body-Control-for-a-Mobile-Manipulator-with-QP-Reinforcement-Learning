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

class MujocoRosBridge(Node):
    def __init__(self, model, data):
        super().__init__('mujoco_ur5e_bridge')
        self.model = model
        self.data = data
        
        #As a publisher, publish joint states as topic '/joint_states'
        #message type:"sensor_msgs/JointState", topic name: "/joint_states", queue size: 10
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.joint_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) 
                            for i in range(self.model.njnt) if "joint" in mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)]
        
        # create a timer to publish joint states at 2Hz (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.publish_states)

    def publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        msg.position = self.data.qpos.tolist()
        msg.velocity = self.data.qvel.tolist()

        #publish message to topic '/joint_states'
        self.joint_pub.publish(msg)

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
