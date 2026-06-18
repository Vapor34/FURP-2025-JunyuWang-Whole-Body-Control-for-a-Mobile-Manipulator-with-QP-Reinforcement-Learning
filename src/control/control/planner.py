import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from pymoveit2 import MoveIt2
from description.description.objname import ARM_JOINT_NAMES


def main():
    rclpy.init()
    node = Node("moveit2_planner_node")
    moveit2 = MoveIt2(
        node=node, 
        joint_names=ARM_JOINT_NAMES, 
        base_link_name="base_link", 
        end_effector_name="tool0")

    # Example usage: Plan to a target pose
    target_pose = {
        "position": [0.5, 0.0, 0.5],
        "orientation": [0.0, 0.0, 0.0, 1.0]  # Quaternion (x, y, z, w)
    }
    
    plan = moveit2.plan_to_pose(target_pose)
    
    if plan:
        node.get_logger().info("Plan found!")
        # Execute the plan if needed
        # moveit2.execute(plan)
    else:
        node.get_logger().info("No valid plan found.")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()