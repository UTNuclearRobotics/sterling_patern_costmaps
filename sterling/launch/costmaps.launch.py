from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory("sterling"), "config", "params.yaml")
    namespace = "panther/sterling" # Namespaced so topics have namespace/node_name or namespace/topic_name

    return LaunchDescription(
        [
            Node(
                package="sterling",
                executable="local_costmap_builder",
                name="local_costmap_builder",
                namespace=namespace,
                parameters=[params_file],
            ),
            Node(
                package="sterling",
                executable="global_costmap_builder",
                name="global_costmap_builder",
                namespace=namespace,
                parameters=[params_file],
            )
        ]
    )
