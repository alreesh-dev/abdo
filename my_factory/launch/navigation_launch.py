import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node # أضفنا هذا السطر للاحتياط

def generate_launch_description():
    pkg_share = get_package_share_directory('my_factory')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # مسار الخريطة والبارامترات
    map_yaml_file = os.path.join(pkg_share, 'maps', 'warehouse_to_edit.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': params_file,
                'use_sim_time': 'true',
                'autostart': 'true', # تفعيل العقد تلقائياً
            }.items(),
        ),
    ])