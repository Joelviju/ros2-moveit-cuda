from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
import os


def generate_launch_description():

    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    moveit_pkg = get_package_share_directory('my_robot_moveit_config')
    desc_pkg = get_package_share_directory('my_robot_description')

    world_path = os.path.join(moveit_pkg, 'worlds', 'empty.sdf')
    controllers_yaml = os.path.join(moveit_pkg, 'config', 'controllers.yaml')

    # ------------------------------------------------------------------
    # Ignition Gazebo
    # ------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items()
    )

    # ------------------------------------------------------------------
    # Robot description (xacro → URDF)
    # ------------------------------------------------------------------
    robot_description = {
        'robot_description': Command([
            'xacro ',
            os.path.join(desc_pkg, 'urdf', 'ur5.xacro')
        ])
    }

    # ------------------------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # ------------------------------------------------------------------
    # Spawn robot into Ignition
    # ------------------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'leo',
            '-allow_renaming', 'true'
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # ros2_control controller manager
    # ------------------------------------------------------------------
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controllers_yaml,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # Controller spawners (delayed to avoid race conditions)
    # ------------------------------------------------------------------
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen',
            )
        ]
    )

    arm_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'arm_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen',
            )
        ]
    )

    # ------------------------------------------------------------------
    # Launch order
    # ------------------------------------------------------------------
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
