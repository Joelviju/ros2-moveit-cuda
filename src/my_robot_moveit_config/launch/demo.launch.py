from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_desc = get_package_share_directory("my_robot_description")
    pkg_moveit = get_package_share_directory("my_robot_moveit_config")

    # --- Robot description (URDF via xacro) ---
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                os.path.join(pkg_desc, "urdf", "ur5.xacro")
            ]),
            value_type=str,
        )
    }

    # --- Semantic description (SRDF) ---
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                "cat ",
                os.path.join(pkg_moveit, "config", "leo.srdf")
            ]),
            value_type=str,
        )
    }

    # --- Kinematics (ROS 2 params file, OK for move_group) ---
    robot_description_kinematics = os.path.join(
        pkg_moveit, "config", "kinematics.yaml"
    )

    # --- Fake controllers (MoveIt-style, ONLY for move_group) ---
    controllers_yaml = os.path.join(
        pkg_moveit, "config", "fake_controllers.yaml"
    )

    # --- RViz config ---
    rviz_config = os.path.join(
        pkg_moveit, "config", "moveit.rviz"
    )

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # MoveIt move_group (ONLY place controllers go)
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                controllers_yaml,
            ],
        ),

        # RViz (NO controllers, NO kinematics)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                robot_description,
                robot_description_semantic,
            ],
        ),
    ])
