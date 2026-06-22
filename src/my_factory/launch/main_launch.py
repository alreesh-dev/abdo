#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'my_factory'
    pkg_path = get_package_share_directory(package_name)

    # إعدادات استخدام وقت المحاكي لضمان تزامن الـ TF
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # 1. تشغيل المحاكي (Gazebo) وملف الـ Robot State Publisher
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. الجسر (Bridge) - النسخة المستقرة مع إضافة lazy: False
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # --- الإضافة هنا لضمان استقرار النشر ومزامنة الوقت ---
        parameters=[{
            'use_sim_time': use_sim_time,
            'lazy': False
        }],
        # --------------------------------------------------
        arguments=[
            # مزامنة الساعة (Clock)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # التحكم بالحركة (cmd_vel)
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            
            # بيانات الـ Odometry
            '/model/warehouse_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            # بيانات الـ IMU (تُدمج مع عداد العجلات في الـ EKF)
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            
            # حالة المفاصل (لتحريك العجلات في RViz)
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # الحساسات (الليزر)
            '/front_right_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/back_left_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/model/warehouse_robot/odometry', '/odom'),
            ('/front_right_scan', '/scan') # الحساس الأساسي للـ SLAM
        ],
        output='screen'
    )

    # 2.5 مرشح EKF لدمج عداد العجلات مع الـ IMU (ينشر تحويل odom -> base_footprint)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    # 3. الـ SLAM Toolbox - لبناء الخريطة (map)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    # 4. تشغيل RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'view_robot.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo_sim,
        bridge,
        ekf_node,
        rviz,
        # تأخير الـ SLAM قليلاً لضمان استقرار شجرة الـ TF أولاً
        TimerAction(period=7.0, actions=[slam_toolbox])
    ])