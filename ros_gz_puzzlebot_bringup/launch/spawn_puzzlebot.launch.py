from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    spawn_puzzlebot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_puzzlebot_entity",
        # fmt: off
        arguments=[
            "-entity", "puzzlebot",
            "-topic", "/robot_description",
            "-x", LaunchConfiguration("x_pose"),
            "-y", LaunchConfiguration("y_pose"),
            "-z", "0.03", 
        ],
        # fmt: on
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="x_pose",
                default_value="0.0",
                description="Initial X-axis position (meters) for spawning the Puzzlebot",
            ),
            DeclareLaunchArgument(
                name="y_pose",
                default_value="0.0",
                description="Initial Y-axis position (meters) for spawning the Puzzlebot",
            ),
            spawn_puzzlebot_node,
        ]
    )