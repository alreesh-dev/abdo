import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_file = os.path.join(pkg_path, 'worlds', 'fac.world')

    # 2. تشغيل محرك المحاكاة Gazebo Ignition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items() 
    )

    # 3. تشغيل الـ Robot State Publisher (لنشر الـ URDF)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. استدعاء (Spawn) الروبوت داخل العالم
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'warehouse_robot',
                   '-z', '0.15'], 
        output='screen'
    )

    # ملاحظة هندسية: تم حذف قسم الـ Bridge من هنا 
    # لأنك تقوم بتشغيله في ملف الـ main_launch.py
    # تشغيله في مكانين هو سبب الـ Flickering (الرفرفة) في RViz

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        rsp,
        spawn_entity
    ])