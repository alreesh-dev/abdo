import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')
    
    # قراءة ملف الـ xacro الخاص بالشاحنة
    xacro_file = os.path.join(pkg_path, 'urdf', 'truck_bin.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # 🟢 إضافة محول ثابت (Static TF) لربط truck_base_link بالعالم لكي يفهمها RViz فوراً
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'truck_base_link']
    )

    # تشغيل RViz لعرض المجسم وحده
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        static_tf,
        rviz_node
    ])