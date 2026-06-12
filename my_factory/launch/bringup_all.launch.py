import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')

    # 1. استدعاء ملف Gazebo الأساسي
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'gazebo.launch.py'))
    )

    # 2. عقدة الجسر (Bridge) - النسخة الاحترافية لربط الأودومتري والحساسات
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # الحركة
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
            # الأودومتري - ربط المسار الكامل (ضروري جداً ليظهر الـ echo)
            '/model/warehouse_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            
            # الليزر
            '/front_right_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/back_left_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
            # الـ TF - ربط بيانات الوضعية الديناميكية
            '/model/warehouse_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            
            # الـ IMU (الحل السحري لمشكلتك - تم التأكد من الصيغة)
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        remappings=[
            ('/model/warehouse_robot/odometry', '/odom'),
            ('/model/warehouse_robot/tf', '/tf')
        ],
        output='screen'
    )
    # 3. دمج الليزرين (Laser Merger)
    laser_merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[{
            'destination_frame': 'base_link', # تأكد أنه نفس الـ frame الرئيسي للروبوت
            'cloud_destination_topic': '/merged_cloud',
            'scan_destination_topic': '/scan',
            'laserscan_topics': '/front_right_scan /back_left_scan',
            'use_sim_time': True,
            'range_min': 0.3,
            'range_max': 12.0
        }]
    )

    # 4. الـ SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    # 5. تشغيل RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'slam_config.rviz')], # إذا كان عندك ملف إعدادات
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        ros_gz_bridge,
        # حذفنا joint_state_publisher العادي لأننا نأخذه من جازيبو مباشرة عبر الجسر
        TimerAction(period=3.0, actions=[laser_merger_node]),
        TimerAction(period=6.0, actions=[slam_launch]),
        rviz_node
    ])