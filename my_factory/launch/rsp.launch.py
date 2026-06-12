#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name = 'my_factory'
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')

    # معالجة الملف وتحويله
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_raw = robot_description_config.toxml()

    # استخدام الـ LaunchConfiguration لضمان استلام القيمة من الملف الرئيسي
    use_sim_time = LaunchConfiguration('use_sim_time')

    # إعداد عقدة الـ Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time, # هاد السطر هو اللي بربط الزمن بالمحاكي
            'publish_frequency': 50.0,
            'ignore_timestamp': False,
            # إضافة frame_prefix فارغ يضمن عدم حدوث إزاحة بأسماء الـ Links
            'frame_prefix': '' 
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        node_robot_state_publisher
    ])