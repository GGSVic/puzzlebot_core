from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Launch gazebo with the default empty world
    gazebo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        )
    )

    # Launch the spawner
    spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("puzzlebot_bringup"),
                    "launch",
                    "spawn_puzzlebot.launch.py",
                ]
            )
        )
    )

    # Launch state publisher
    joint_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("puzzlebot_bringup"),
                    "launch",
                    "state_publisher.launch.py",
                ]
            )
        )
    )

    return LaunchDescription([gazebo_launcher, spawner, joint_state_publisher])
