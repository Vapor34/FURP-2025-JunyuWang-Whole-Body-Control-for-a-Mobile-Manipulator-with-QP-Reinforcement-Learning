import threading
from pathlib import Path

import mujoco # type: ignore
import mujoco.viewer # type: ignore

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from description.description.robot_constants import ARM_JOINT_NAMES


class MujocoSimulationBridge(Node):
    def __init__(self, model, data):
        super().__init__('mujoco_ur5e_bridge')
        self.model = model
        self.data = data

        self.joint_names = list(ARM_JOINT_NAMES)
        self.joint_position_addresses = []
        self.joint_velocity_addresses = []
        
        # Publish the current simulated joint states for robot_state_publisher and MoveIt.
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        
        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name
            )

            if joint_id < 0:
                raise RuntimeError(f"Joint not found in MuJoCo model: {joint_name}")

            self.joint_position_addresses.append(self.model.jnt_qposadr[joint_id])
            self.joint_velocity_addresses.append(self.model.jnt_dofadr[joint_id])

        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            self.handle_joint_trajectory, 
            10
        )
        self.joint_state_timer = self.create_timer(0.02, self.publish_joint_states)

    def handle_joint_trajectory(self, msg):
        if not msg.points:
            return

        target_point = msg.points[-1]
        for commanded_index, joint_name in enumerate(msg.joint_names):
            if joint_name not in self.joint_names:
                continue

            if commanded_index >= len(target_point.positions):
                continue

            simulated_index = self.joint_names.index(joint_name)
            position_address = self.joint_position_addresses[simulated_index]
            self.data.qpos[position_address] = target_point.positions[commanded_index]

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        msg.position = [
            float(self.data.qpos[position_address])
            for position_address in self.joint_position_addresses
        ]

        msg.velocity = [
            float(self.data.qvel[velocity_address])
            for velocity_address in self.joint_velocity_addresses
        ]

        self.joint_state_publisher.publish(msg)

def ros_spin_thread(node):
    rclpy.spin(node)

def get_model_path():
    description_share = Path(get_package_share_directory("description"))
    return description_share / "assets" / "mujoco" / "ur5e.xml"

def main():
    rclpy.init()

    model_path = get_model_path()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # 3. 启动 ROS 桥接节点（放在后台线程，防止阻塞 MuJoCo 的可视化窗口）
    ros_node = MujocoSimulationBridge(model, data)
    ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    ros_thread.start()

    print("MuJoCo simulation and ROS 2 Bridge started.")
    mujoco.viewer.launch(model, data)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
