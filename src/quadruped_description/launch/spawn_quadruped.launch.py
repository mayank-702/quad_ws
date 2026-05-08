from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # 🔹 Xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("quadruped_description"),
        "urdf",
        "quadruped.xacro"
    ])

    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_file]),
        value_type=str
    )

    # 🔹 Controllers config
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("quadruped_description"),
        "config",
        "controllers.yaml"
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare("quadruped_description"),
        "worlds",
        "home.world"
    ])

    # 🔹 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_path
            }.items()
    )

    # 🔹 Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
        output="screen"
    )

    # 🔹 ros2_control
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {'robot_description': robot_description},
    #         controllers_yaml
    #     ],
    #     output="screen"
    # )

    # 🔹 Spawn robot (DELAYED → Gazebo needs time)
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'quadruped',
                    '-z', '1.0'
                ],
                output='screen'
            )
        ]
    )

    # 🔹 Controllers (DELAYED → after spawn)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
        output="screen",
    )

    delayed_controllers = TimerAction(
        period=6.0,
        actions=[
            joint_state_broadcaster_spawner,
            trajectory_controller_spawner
        ]
    )

    return LaunchDescription([
        gazebo,
        rsp,
        # controller_manager,
        spawn,
        delayed_controllers
    ])
