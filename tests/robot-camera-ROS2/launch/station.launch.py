from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = str(Path(get_package_share_directory("station_bridge")) / "config" / "station.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        Node(
            package="station_bridge",
            executable="station_bridge_node",
            name="station_bridge",
            output="screen",
            parameters=[LaunchConfiguration("params")],
        ),
    ])
