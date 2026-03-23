'''from launch import LaunchDescription
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
'''

'''
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():

    moveit_pkg = get_package_share_directory('my_robot_moveit_config')
    desc_pkg = get_package_share_directory('my_robot_description')

    # ------------------------------------------------------------
    # Fix mesh loading in Ignition
    # ------------------------------------------------------------
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        f"{get_package_prefix('ur_description')}/share/:"
    )

    # ------------------------------------------------------------
    # World
    # ------------------------------------------------------------
    custom_world = os.path.join(moveit_pkg, "worlds", "empty.sdf")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=[('ign_args', [f'-r {custom_world}'])]
    )

    # ------------------------------------------------------------
    # MoveIt Config Builder (USING YOUR ur5.xacro)
    # ------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("leo", package_name="my_robot_moveit_config")
        .robot_description(file_path=os.path.join(desc_pkg, 'urdf', 'ur5.xacro'))
        .robot_description_semantic(file_path='config/leo.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=os.path.join(moveit_pkg, 'config', 'moveit_cpp.yaml'))
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    config_dict = moveit_config.to_dict()
    config_dict.update({"use_sim_time": True})

    # ------------------------------------------------------------
    # Spawn Robot (same structure as your working file)
    # ------------------------------------------------------------
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'leo',
            '-allow_renaming', 'true'
        ],
    )

    # ------------------------------------------------------------
    # Robot State Publisher
    # ------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description,
                    {'use_sim_time': True}],
        output="screen"
    )

    # ------------------------------------------------------------
    # Move Group
    # ------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[config_dict]
    )

    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("my_robot_moveit_config"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_path]
    )

    # ------------------------------------------------------------
    # Controllers
    # ------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'joint_state_broadcaster',
            "--controller-manager",
            "/controller_manager"
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller'],
    )

    # ------------------------------------------------------------
    # Launch Order (following your working structure style)
    # ------------------------------------------------------------
    return LaunchDescription([
        ignition_spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        move_group_node,
        gzserver_cmd,
        rviz_node,
        arm_controller_spawner,
    ])
'''
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():

    moveit_pkg = get_package_share_directory('my_robot_moveit_config')
    desc_pkg = get_package_share_directory('my_robot_description')

    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        f"{get_package_prefix('ur_description')}/share/:"
    )

    # ---------------- Gazebo ----------------
    custom_world = os.path.join(moveit_pkg, "worlds", "empty.sdf")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=[('ign_args', [f'-r {custom_world}'])]
    )

    # ---------------- MoveIt Config ----------------
    moveit_config = (
        MoveItConfigsBuilder("leo", package_name="my_robot_moveit_config")
        .robot_description(file_path=os.path.join(desc_pkg, 'urdf', 'ur5.xacro'))
        .robot_description_semantic(file_path='config/leo.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')   # ✅ CRITICAL
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path='config/moveit_cpp.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    config_dict = moveit_config.to_dict()
    config_dict.update({"use_sim_time": True})

    # ---------------- Spawn Robot ----------------
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'manipulator',
            '-allow_renaming', 'true'
        ],
    )

    # ---------------- Robot State Publisher ----------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description,
                    {'use_sim_time': True}],
        output="screen"
    )
    move_to_pose_node = Node(
        package="my_robot_cpp_moveit",
        #executable="move_to_pose",
        executable="cpu_planning_benchmark_CUDA",
        output="screen",
        parameters=[config_dict],   # THIS LINE FIXES EVERYTHING
    )
    # ---------------- Controllers ----------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller'],
    )

    # ---------------- Move Group ----------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[config_dict],   # ✅ single merged dict
    )

    # ---------------- RViz ----------------
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("my_robot_moveit_config"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_path],
        parameters=[config_dict],   # ✅ VERY IMPORTANT
    )

    return LaunchDescription([
        gzserver_cmd,
        ignition_spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
        move_to_pose_node,
    ])