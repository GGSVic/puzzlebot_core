from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, PythonExpression


def generate_launch_description():

    # World file
    world_path = PathJoinSubstitution(
        [
            FindPackageShare("ros_gz_puzzlebot_bringup"), 
            "worlds", 
            "empty.sdf"
        ]
    )

    # Launch gazebo with the default empty world
    gazebo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ), 
        launch_arguments={
            'gz_args': PythonExpression(["'-r ' + '", world_path, "'"])
        }.items()
    )

    # Launch the spawner
    spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_puzzlebot_bringup"),
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
                    FindPackageShare("ros_gz_puzzlebot_bringup"),
                    "launch",
                    "state_publisher.launch.py",
                ]
            )
        )
    )
    

    return LaunchDescription([gazebo_launcher, spawner, joint_state_publisher])