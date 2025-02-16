import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ompl_planning_yaml = os.path.join(
        get_package_share_directory('diana_7_moveit'),
        'config',
        'ompl_planning.yaml'
    )
    moveit_config = (
        MoveItConfigsBuilder("diana_v2", package_name="diana_7_moveit")
        .robot_description(file_path="config/diana_v2.urdf.xacro")  # 确保 diana_v2.urdf.xacro 文件存在
        
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        
        .to_moveit_configs()
    )
    diana_decription_package_name = 'diana_description'
    # urdf_name = "diana_v2.urdf"
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=diana_decription_package_name).find(diana_decription_package_name)

    rviz_file = os.path.join(pkg_share, 'rviz/move_group.rviz')

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

    diana_7_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diana_7_controller'],
        output='screen'
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["--display-config", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )


    ld.add_action(robot_state_publisher_node)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(diana_7_controller_node)
    ld.add_action(run_move_group_node)
    ld.add_action(rviz2_node)
    return ld
