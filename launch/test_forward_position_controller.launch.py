from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CONTROL_PKG = "rr1_control"

CONFIG_NAME = "tests_grip"
CONFIG_FILE = f"{CONFIG_NAME}.yaml"

def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare(CONTROL_PKG), "config", CONFIG_FILE
        ])

    topic = LaunchConfiguration('topic')
    declare_topic_cmd = DeclareLaunchArgument(
        'topic',
        default_value='/rr1/forward_position_controller/commands',
        description='')
    
    return LaunchDescription(
        [
            declare_topic_cmd,
            Node(
                package="rr1_control",
                executable="test_forward_position_controller.py",
                name="forward_position_controller_test_publisher",
                parameters=[
                    {"topic_name": topic},
                    config_path
                ],
                output="both"
            )
        ]
    )
