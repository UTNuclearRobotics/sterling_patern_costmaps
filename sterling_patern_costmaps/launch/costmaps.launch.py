import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory("sterling_patern_costmaps"), "config", "params.yaml")
    sterling_patern_costmaps_dir = get_package_share_directory("sterling_patern_costmaps")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="panther/sterling",
        description="Add namespace to all launched nodes.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )
    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2.",
        choices=["true", "false"],
    )
    declare_rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(sterling_patern_costmaps_dir, "config", "panther_sim.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_use_sim_time_arg,
            declare_use_rviz_arg,
            declare_rviz_config_file_arg,
            Node(
                package="sterling_patern_costmaps",
                executable="local_costmap_builder",
                name="local_costmap_builder",
                namespace=namespace,
                parameters=[params_file],
            ),
            Node(
                package="sterling_patern_costmaps",
                executable="global_costmap_builder",
                name="global_costmap_builder",
                namespace=namespace,
                parameters=[params_file],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz_mapping",
                arguments=[
                    "-d",
                    PathJoinSubstitution([rviz_config_file]),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                ('/goal_pose', '/panther/goal_pose'),
                ],
                output="screen",
            ),
        ]
    )