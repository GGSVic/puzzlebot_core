from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
        namespace="puzzlebot",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_path]), value_type=str
                ),
            }
        ],
        remappings=[
            ("/tf", "/tf"),
            ("/tf_static", "/tf_static"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
                choices=["true", "false"],
            ),
            robot_state_publisher_node,
        ]
    )
