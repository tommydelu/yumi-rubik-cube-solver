from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("yumi_cube")) / "config" / "parameters.yaml"

    return LaunchDescription([
        Node(
            package="yumi_cube",
            executable="yumi_cube_node",
            name="yumi_cube_node",
            output="screen",
            parameters=[str(config)],
        ),
        Node(
            package="yumi_cube",
            executable="sequence_node",
            name="sequence_parser_node",
            output="screen",
        ),
    ])
