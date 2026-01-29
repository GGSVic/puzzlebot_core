from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # Retrieve launch configuration values
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Build absolute path to RViz configuration file
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("puzzlebot_description"),
            "rviz",
            [EnvironmentVariable("PUZZLEBOT_MODEL"), ".rviz"],
        ]
    )

    # RViz2 node with custom configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="Use simulation time instead of wall clock time",
                choices=["true", "false"],
            ),
            rviz_node,
        ]
    )
