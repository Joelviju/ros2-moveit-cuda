from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot_moveit_config')
    world_path = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    # Launch Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py')),
        launch_arguments={
            'ign_args': f'-r {world_path}'
        }.items()
    )

    # Robot description (xacro → urdf)
    robot_description = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([
                get_package_share_directory('my_robot_description'),
                'urdf',
                'ur5.xacro'
            ])
        ])
    }

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Spawn robot into Ignition
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

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
