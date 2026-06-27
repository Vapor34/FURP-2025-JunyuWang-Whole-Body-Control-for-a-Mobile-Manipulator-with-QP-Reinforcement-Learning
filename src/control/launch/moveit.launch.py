from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = Path(get_package_share_directory("description"))
    urdf_path = description_share / "assets" / "xacro" / "ur.urdf.xacro"

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=str(urdf_path),
            mappings={"ur_type": "ur5e", "name": "ur"},
        )
        .robot_description_semantic(
            file_path=Path("srdf") / "ur.srdf.xacro",
            mappings={"name": "ur"},
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
            load_all=False,
        )
        .to_moveit_configs()
    )
    moveit_parameters = moveit_config.to_dict()
    moveit_parameters["moveit_manage_controllers"] = False
    moveit_parameters["moveit_simple_controller_manager"] = {
        "controller_names": ["joint_trajectory_controller"],
        "joint_trajectory_controller": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        },
    }

    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = (
        Path(get_package_share_directory("ur_moveit_config"))
        / "config"
        / "moveit.rviz"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the MoveIt planning plugin",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                moveit_parameters,
                {"allow_trajectory_execution": True},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="screen",
            arguments=["-d", str(rviz_config)],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(launch_rviz),
        ),
    ])
