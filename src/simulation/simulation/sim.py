import threading
import time
from pathlib import Path

import mujoco  # type: ignore
import mujoco.viewer  # type: ignore
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from description.robot_constants import ARM_JOINT_NAMES


ACTION_NAME = "/joint_trajectory_controller/follow_joint_trajectory"
COMMAND_TOPIC = "/joint_trajectory_controller/joint_trajectory"
CONTROL_PERIOD = 0.01
DEFAULT_GOAL_TOLERANCE = 0.05
DEFAULT_SETTLE_TIME = 2.0


def duration_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class MujocoSimulationBridge(Node):
    def __init__(self, model, data):
        super().__init__("mujoco_ur5e_bridge")
        self.model = model
        self.data = data
        self.data_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.trajectory_state_lock = threading.Lock()
        self.trajectory_active = False
        self.declare_parameter("headless", False)

        self.joint_names = list(ARM_JOINT_NAMES)
        self.joint_name_to_index = {
            name: index for index, name in enumerate(self.joint_names)
        }
        self.joint_position_addresses = []
        self.joint_velocity_addresses = []
        self.joint_actuator_ids = []

        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )
            if joint_id < 0:
                raise RuntimeError(f"Joint not found in MuJoCo model: {joint_name}")

            self.joint_position_addresses.append(self.model.jnt_qposadr[joint_id])
            self.joint_velocity_addresses.append(self.model.jnt_dofadr[joint_id])

            actuator_id = next(
                (
                    candidate_id
                    for candidate_id in range(self.model.nu)
                    if self.model.actuator_trnid[candidate_id, 0] == joint_id
                ),
                None,
            )
            if actuator_id is None:
                raise RuntimeError(
                    f"Position actuator not found for MuJoCo joint: {joint_name}"
                )
            self.joint_actuator_ids.append(actuator_id)

        self.data.ctrl[self.joint_actuator_ids] = self.data.qpos[
            self.joint_position_addresses
        ]

        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory,
            COMMAND_TOPIC,
            self.handle_joint_trajectory,
            10,
        )
        self.joint_state_timer = self.create_timer(0.02, self.publish_joint_states)

        action_callback_group = ReentrantCallbackGroup()
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
            execute_callback=self.execute_trajectory,
            goal_callback=self.handle_trajectory_goal,
            cancel_callback=self.handle_trajectory_cancel,
            callback_group=action_callback_group,
        )

    def handle_trajectory_goal(self, goal_request):
        trajectory = goal_request.trajectory
        rejection_reason = self.validate_trajectory(trajectory)
        if rejection_reason:
            self.get_logger().error(f"Rejecting trajectory: {rejection_reason}")
            return GoalResponse.REJECT

        with self.trajectory_state_lock:
            if self.trajectory_active:
                self.get_logger().warning("Rejecting trajectory: controller is busy")
                return GoalResponse.REJECT
            self.trajectory_active = True

        return GoalResponse.ACCEPT

    def handle_trajectory_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def validate_trajectory(self, trajectory):
        if not trajectory.joint_names:
            return "joint_names is empty"
        if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
            return "joint_names contains duplicates"
        if any(name not in self.joint_name_to_index for name in trajectory.joint_names):
            return "trajectory contains a joint that is not in the MuJoCo model"
        if not trajectory.points:
            return "trajectory has no points"

        previous_time = -1.0
        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                return "a trajectory point has the wrong number of positions"
            point_time = duration_seconds(point.time_from_start)
            if point_time < 0.0 or point_time <= previous_time:
                return "time_from_start values must be strictly increasing"
            previous_time = point_time

        return None

    def handle_joint_trajectory(self, msg):
        rejection_reason = self.validate_trajectory(msg)
        if rejection_reason:
            self.get_logger().error(f"Ignoring trajectory topic command: {rejection_reason}")
            return

        with self.trajectory_state_lock:
            if self.trajectory_active:
                self.get_logger().warning(
                    "Ignoring trajectory topic command while an action is active"
                )
                return

        self.set_joint_targets(msg.joint_names, msg.points[-1].positions)

    def set_joint_targets(self, joint_names, positions):
        with self.data_lock:
            for joint_name, position in zip(joint_names, positions):
                simulated_index = self.joint_name_to_index[joint_name]
                actuator_id = self.joint_actuator_ids[simulated_index]
                self.data.ctrl[actuator_id] = position

    def get_joint_positions(self, joint_names):
        with self.data_lock:
            return [
                float(
                    self.data.qpos[
                        self.joint_position_addresses[
                            self.joint_name_to_index[joint_name]
                        ]
                    ]
                )
                for joint_name in joint_names
            ]

    def hold_current_positions(self, joint_names):
        self.set_joint_targets(joint_names, self.get_joint_positions(joint_names))

    def publish_trajectory_feedback(self, goal_handle, joint_names, desired_positions):
        actual_positions = self.get_joint_positions(joint_names)
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(joint_names)
        feedback.desired = JointTrajectoryPoint(
            positions=list(desired_positions),
        )
        feedback.actual = JointTrajectoryPoint(
            positions=actual_positions,
        )
        feedback.error = JointTrajectoryPoint(
            positions=[
                desired - actual
                for desired, actual in zip(desired_positions, actual_positions)
            ],
        )
        goal_handle.publish_feedback(feedback)
        return actual_positions

    def canceled_result(self, goal_handle, joint_names):
        self.hold_current_positions(joint_names)
        goal_handle.canceled()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory canceled"
        return result

    def execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        result = FollowJointTrajectory.Result()

        try:
            previous_positions = self.get_joint_positions(joint_names)
            previous_time = 0.0
            start_time = time.monotonic()

            for point in trajectory.points:
                target_positions = list(point.positions)
                target_time = duration_seconds(point.time_from_start)
                segment_duration = target_time - previous_time

                while True:
                    if goal_handle.is_cancel_requested:
                        return self.canceled_result(goal_handle, joint_names)
                    if self.stop_event.is_set() or not rclpy.ok():
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                        result.error_string = "Simulation is shutting down"
                        return result

                    elapsed = time.monotonic() - start_time
                    if elapsed >= target_time:
                        break

                    alpha = min(
                        1.0,
                        max(0.0, (elapsed - previous_time) / segment_duration),
                    )
                    desired_positions = [
                        start + alpha * (target - start)
                        for start, target in zip(
                            previous_positions,
                            target_positions,
                        )
                    ]
                    self.set_joint_targets(joint_names, desired_positions)
                    self.publish_trajectory_feedback(
                        goal_handle,
                        joint_names,
                        desired_positions,
                    )
                    time.sleep(min(CONTROL_PERIOD, target_time - elapsed))

                self.set_joint_targets(joint_names, target_positions)
                previous_positions = target_positions
                previous_time = target_time

            goal_tolerances = {
                tolerance.name: tolerance.position
                for tolerance in goal_handle.request.goal_tolerance
                if tolerance.name in joint_names and tolerance.position > 0.0
            }
            requested_settle_time = duration_seconds(
                goal_handle.request.goal_time_tolerance
            )
            settle_time = requested_settle_time or DEFAULT_SETTLE_TIME
            settle_deadline = time.monotonic() + settle_time
            final_positions = list(trajectory.points[-1].positions)

            while True:
                if goal_handle.is_cancel_requested:
                    return self.canceled_result(goal_handle, joint_names)

                actual_positions = self.publish_trajectory_feedback(
                    goal_handle,
                    joint_names,
                    final_positions,
                )
                within_tolerance = all(
                    abs(target - actual)
                    <= goal_tolerances.get(joint_name, DEFAULT_GOAL_TOLERANCE)
                    for joint_name, target, actual in zip(
                        joint_names,
                        final_positions,
                        actual_positions,
                    )
                )
                if within_tolerance:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Trajectory executed successfully"
                    return result

                if time.monotonic() >= settle_deadline:
                    goal_handle.abort()
                    result.error_code = (
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    )
                    result.error_string = "Final joint tolerance was not reached"
                    return result

                time.sleep(CONTROL_PERIOD)
        finally:
            with self.trajectory_state_lock:
                self.trajectory_active = False

    def publish_joint_states(self):
        with self.data_lock:
            positions = [
                float(self.data.qpos[address])
                for address in self.joint_position_addresses
            ]
            velocities = [
                float(self.data.qvel[address])
                for address in self.joint_velocity_addresses
            ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = velocities
        self.joint_state_publisher.publish(msg)

    def stop(self):
        self.stop_event.set()


def get_model_path():
    description_share = Path(get_package_share_directory("description"))
    return description_share / "assets" / "mujoco" / "ur5e.xml"


def run_simulation(ros_node, viewer=None):
    while rclpy.ok() and (viewer is None or viewer.is_running()):
        step_started = time.monotonic()
        with ros_node.data_lock:
            mujoco.mj_step(ros_node.model, ros_node.data)
        if viewer is not None:
            viewer.sync()

        remaining_step_time = ros_node.model.opt.timestep - (
            time.monotonic() - step_started
        )
        if remaining_step_time > 0.0:
            time.sleep(remaining_step_time)


def main():
    rclpy.init()

    model = mujoco.MjModel.from_xml_path(str(get_model_path()))
    data = mujoco.MjData(model)
    ros_node = MujocoSimulationBridge(model, data)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(ros_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        ros_node.get_logger().info("MuJoCo simulation and ROS 2 bridge started")
        if ros_node.get_parameter("headless").value:
            run_simulation(ros_node)
        else:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                run_simulation(ros_node, viewer)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.stop()
        executor.shutdown(timeout_sec=2.0)
        ros_thread.join(timeout=2.0)
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
