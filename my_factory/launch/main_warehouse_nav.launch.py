import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')
    rviz_config_path = os.path.join(get_package_share_directory('my_factory'), 'rviz', 'nav2_default_view.rviz')
    
    # تحديد المسارات الصحيحة للملفات داخل حزمة share
    params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    custom_bt_xml = os.path.expanduser('~/abdo/src/my_factory/behavior_trees/simple_nav.xml')

    # 1. تشغيل بيئة المحاكاة جازيبو
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'gazebo.launch.py'))
    )

    # 2. جسر الاتصال بين ROS 2 و Ignition Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/warehouse_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/warehouse_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/front_right_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/back_left_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/model/warehouse_robot/odometry', '/odom'),
            ('/model/warehouse_robot/tf', '/tf')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 3. دمج بيانات الليزر مع ضبط نطاق الرؤية والأبعاد بدقة لنسخة Humble لمنع طيران الإشارات
    laser_merger = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'destination_frame': 'base_footprint', 
            'scan_destination_topic': '/scan',
            'laserscan_topics': '/front_right_scan /back_left_scan',
            # البارامترات الحيوية لضبط زوايا وأبعاد أشعة الليزر المدمجة:
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.01745,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 12.0,
        }]
    )

    # 4. عقد نظام الملاحة (Nav2 Nodes)
    nav_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file, 
                {
                    'use_sim_time': True, 
                    'yaml_filename': os.path.expanduser('~/abdo/src/my_factory/maps/warehouse_to_edit.yaml')
                }
            ]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file, 
                {
                    'use_sim_time': True,
                    'default_bt_xml_filename': custom_bt_xml,
                    'default_nav_to_pose_bt_xml': custom_bt_xml,
                    'default_nav_through_poses_bt_xml': custom_bt_xml
                }
            ]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}]
        ),
        Node(
             package='nav2_waypoint_follower',
             executable='waypoint_follower',
             name='waypoint_follower',
             output='screen',
             parameters=[params_file, {'use_sim_time': True}]
        ),
        
    ]

    # 5. مدير دورة الحياة (Lifecycle Manager)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'autostart': True, 
            'node_names': [
                'map_server', 
                'amcl', 
                'planner_server', 
                'controller_server', 
                'bt_navigator', 
                'behavior_server',
                'waypoint_follower'
            ]
        }]
    )

    # 6. تشغيل واجهة RViz2 بتأخير بسيط لضمان جاهزية العقد
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        laser_merger,
        *nav_nodes,
        lifecycle_manager,
        TimerAction(period=12.0, actions=[rviz])
    ])