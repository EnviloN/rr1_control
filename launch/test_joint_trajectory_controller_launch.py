import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

CONTROL_PKG = "rr1_control"

CONFIG_NAME = "tests_grip"
CONFIG_FILE = f"{CONFIG_NAME}.yaml"

def generate_launch_description():
    control_pkg = get_package_share_directory(CONTROL_PKG)
    config_path = os.path.join(control_pkg, "config", CONFIG_FILE)

    return LaunchDescription(
        [
            Node(
                package="rr1_control",
                executable="test_joint_trajectory_controller.py",
                name="joint_trajectory_controller_test_publisher",
                parameters=[config_path],
                output="both"
            )
        ]
    )
