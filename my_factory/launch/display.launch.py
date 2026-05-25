import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # 1. تحديد المسارات الأساسية
    pkg_path = get_package_share_directory('my_factory')
    default_rviz_config_path = os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')

    # 2. تعريف ملف الـ URDF الأساسي (استخدام xacro لمعالجته)
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    
    # 3. عقدة Robot State Publisher
    # تقوم بتحويل الـ URDF إلى تحويلات TF وتشرها للـ ROS
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # 4. عقدة Joint State Publisher GUI
    # تظهر لك واجهة منزلقات (Sliders) للتحكم بالمفاصل يدوياً للتأكد من حركتها
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # 5. تشغيل RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])