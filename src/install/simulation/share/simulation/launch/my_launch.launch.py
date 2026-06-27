from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulation',
            executable='sim',
            name='mujoco_ur5e_bridge',
            output='screen'
        )
    ])