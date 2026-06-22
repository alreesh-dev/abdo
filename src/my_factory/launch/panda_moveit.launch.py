#!/usr/bin/env python3
"""
Bring up the Panda manipulator in the warehouse world with MoveIt + ign_ros2_control.

Requires sourcing the workspace that provides ign_ros2_control — see
config/abdo.env.example and env/setup_abdo.bash.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')
    panda_moveit_pkg = get_package_share_directory('moveit_resources_panda_moveit_config')

    world_file = os.path.join(pkg_path, 'worlds', 'fac.world')
    xacro_file = os.path.join(pkg_path, 'urdf', 'panda_ign.urdf.xacro')

    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda', package_name='moveit_resources_panda_moveit_config')
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(file_path=os.path.join(pkg_path, 'config', 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'panda', '-z', '0.0'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['panda_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    hand_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['panda_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    rviz_config = os.path.join(panda_moveit_pkg, 'launch', 'moveit.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        bridge,
        spawn,
        move_group,
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=arm_spawner, on_exit=[hand_spawner])),
    ])
