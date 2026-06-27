from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command


def generate_launch_description():
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('description'),
            'assets',
            'xacro',
            'ur.urdf.xacro',
        ]),
        ' ur_type:=ur5e',
        ' name:=ur',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run MuJoCo without opening its viewer',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
            }],
        ),
        Node(
            package='simulation',
            executable='sim',
            name='mujoco_ur5e_bridge',
            output='screen',
            parameters=[{'headless': LaunchConfiguration('headless')}],
        )
    ])
