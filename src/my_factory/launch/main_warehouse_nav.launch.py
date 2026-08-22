import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# 🟢 إضافة مكتبات MoveIt لكي يفهمها RViz
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')
    rviz_config_path = os.path.join(get_package_share_directory('my_factory'), 'rviz', 'nav2_default_view.rviz')

    params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    custom_bt_xml = os.path.join(pkg_path, 'behavior_trees', 'simple_nav.xml')

    # 🟢 تهيئة إعدادات الذراع لتمريرها إلى RViz لكي يظهر القوائم والمجسم
    xacro_file = os.path.join(pkg_path, 'urdf', 'panda_ign.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda', package_name='moveit_resources_panda_moveit_config')
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(file_path=os.path.join(pkg_path, 'config', 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # 1. تشغيل بيئة المحاكاة جازيبو
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'gazebo.launch.py'))
    )

    # إضافة أمر استدعاء ملف حقن الذراع الخاص بـ MoveIt
    panda_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'spawn_panda_only.launch.py'))
    )

    # 2. جسر الاتصال بين ROS 2 و Ignition Gazebo (تم دمجه بالكامل مع الكاميرات)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/warehouse_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/front_right_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/back_left_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
            # 📸 مسارات الكاميرا المدمجة (لونية، عمق، وسحابة نقاط) لتفادي انهيار النظام
            '/camera/image_raw/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/image_raw/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/model/warehouse_robot/odometry', '/odom'),
            ('/front_right_scan', '/scan'),  # الليدار الرئيسي المرفوع على العمود
            # إعادة تسمية مسارات الكاميرا لتطابق أكواد الرؤية والملاحة
            ('/camera/image_raw/image', '/camera/image_raw'),
            ('/camera/image_raw/points', '/camera/points')
        ],
        parameters=[{
            'use_sim_time': True,
            'lazy': False  # لضمان استقرار البث ومنع أي رفرفة أو انقطاع
        }],
        output='screen'
    )

    # 3. مرشح EKF لدمج عداد العجلات مع الـ IMU (ينشر تحويل odom -> base_footprint)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config', 'ekf.yaml'), {'use_sim_time': True}]
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
                    'yaml_filename': os.path.join(pkg_path, 'maps', 'warehouse_to_edit.yaml')
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
        # 🟢 السر هنا: قمنا بحقن إعدادات الذراع (moveit_config) داخل RViz
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        ekf_node,
        
        panda_arm,  
        
        *nav_nodes,
        lifecycle_manager,
        TimerAction(period=12.0, actions=[rviz])
    ])