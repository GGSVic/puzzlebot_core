from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:

    # Retrieve launch configuration values
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Build path to urdf.xacro file
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("puzzlebot_description"),
            "urdf",
            "puzzlebot.urdf.xacro",
        ]
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="puzzlebot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_path]), value_type=str
                ),
            }
        ],
        output="screen",
    )

    # Build absolute path to RViz configuration file
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("puzzlebot_bringup"),
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
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                name="rviz",
                default_value="true",
                description="Enable RViz2 visualization for robot model and sensor data",
                choices=["true", "false"],
            ),
            robot_state_publisher_node,
            rviz_node,
        ]
    )
