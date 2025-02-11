import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    diana_decription_package_name = 'diana_description'
    # urdf_name = "diana_v2.urdf"
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=diana_decription_package_name).find(diana_decription_package_name)

    rviz_file = os.path.join(pkg_share, 'rviz/diana7.rviz')

    diana_xacro_path = os.path.join(pkg_share, 'urdf/diana7.urdf.xacro')
    diana7_description = xacro.process_file(diana_xacro_path).toprettyxml(indent="  ")

    diana7_controllers = PathJoinSubstitution([FindPackageShare('diana_bringup'), 'config', 'controllers.yaml'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": diana7_description}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[diana7_controllers,
                    {"robot_description": diana7_description},
        ],
    )

    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["--display-config", rviz_file]
    )
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(joint_trajectory_controller_node)
    ld.add_action(rviz2_node)
    return ld
