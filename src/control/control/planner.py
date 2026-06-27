import rclpy
from rclpy.node import Node

from pymoveit2 import MoveIt2

from description.robot_constants import ARM_JOINT_NAMES


def main():
    rclpy.init()
    node = Node("moveit2_planner")

    node.declare_parameter("position", [0.5, 0.0, 0.5])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("planner_id", "")
    node.declare_parameter("max_velocity", 0.5)
    node.declare_parameter("max_acceleration", 0.5)
    node.declare_parameter("execute", True)

    moveit2 = MoveIt2(
        node=node,
        joint_names=ARM_JOINT_NAMES,
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur_manipulator",
    )
    moveit2.planner_id = node.get_parameter("planner_id").value
    moveit2.max_velocity = node.get_parameter("max_velocity").value
    moveit2.max_acceleration = node.get_parameter("max_acceleration").value

    position = tuple(node.get_parameter("position").value)
    quat_xyzw = tuple(node.get_parameter("quat_xyzw").value)
    node.get_logger().info(
        f"Planning to position={position}, quat_xyzw={quat_xyzw}"
    )

    try:
        trajectory = moveit2.plan(
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id="base_link",
            target_link="tool0",
        )

        if trajectory is None:
            node.get_logger().error("MoveIt could not find a valid trajectory.")
        else:
            node.get_logger().info(
                f"Plan found with {len(trajectory.points)} trajectory points."
            )
            if node.get_parameter("execute").value:
                node.get_logger().info("Executing trajectory...")
                moveit2.execute(trajectory)
                if moveit2.wait_until_executed():
                    node.get_logger().info("Trajectory execution succeeded.")
                else:
                    node.get_logger().error("Trajectory execution failed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
