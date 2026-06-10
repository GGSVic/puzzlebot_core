from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    spawn_puzzlebot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_puzzlebot_entity",
        arguments=[
            "-entity", "puzzlebot",
            "-topic", "/robot_description",
            "-x", LaunchConfiguration("x_pose"),
            "-y", LaunchConfiguration("y_pose"),
            "-z", "0.05",
        ],
        output="screen",
    )

    # Build absolute path to bridge configuration file
    bridge_config_path = PathJoinSubstitution(
        [
            FindPackageShare("ros_gz_puzzlebot_bringup"),
            "config",
            "full.yaml",        
        ]
    )

    # Ros bridge 
    ros_bridge = Node(
        package="ros_gz_bridge", 
        executable="parameter_bridge", 
        name="bridge", 
        parameters=[{"config_file" : bridge_config_path}],
        output="screen"
    )

    # Publishes /joint_states from ros2_control
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Forklift position controller
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ros_gz_puzzlebot_description'),
            'config',
            'forklift_controllers.yaml',
        ]
    )

    forklift_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forklift_controller", 
            "--param-file", 
            robot_controllers,
            ],
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

            # Spawn robot in Gazebo
            spawn_puzzlebot_node,
            ros_bridge,

            # Wait until robot is spawned before loading JS broadcaster
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_puzzlebot_node,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),

            # Wait until broadcaster is active before loading forklift controller
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[forklift_controller_spawner],
                )
            ),
        ]
    )