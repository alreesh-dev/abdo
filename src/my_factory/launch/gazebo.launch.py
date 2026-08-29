import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = os.path.join(pkg_path, 'worlds', 'fac.world')

    # 1. تشغيل محرك المحاكاة Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items() 
    )

    # 2. تشغيل الـ Robot State Publisher للمنصة القديمة
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. معالجة ونشر وصف الحاوية الجديدة (truck_bin) عبر robot_state_publisher خاص بها لكي تظهر بعجلاتها
    truck_xacro_file = os.path.join(pkg_path, 'urdf', 'truck_bin.urdf.xacro')
    truck_description = {'robot_description': xacro.process_file(truck_xacro_file).toxml()}

    truck_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='factory_truck',
        output='screen',
        parameters=[truck_description, {'use_sim_time': use_sim_time}]
    )

    # 4. استدعاء المنصة القديمة
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'warehouse_robot',
                   '-x', '-17.0',
                   '-y', '-10.5',
                   '-z', '0.15'],
        output='screen'
    )

    # 5. استدعاء وحقن الحاوية الجديدة مع تعديل الارتفاع z إلى 0.1 لتناسب سماكة الأرضية الخشبية ودوران 90 درجة
    spawn_truck = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/factory_truck/robot_description',
                   '-name', 'factory_truck',
                   '-x', '-20.9',   # إحداثي X
                   '-y', '-16.4',   # إحداثي Y
                   '-z', '0.1',     # 👈 ارتفاع متوافق مع سماكة الأرضية الخشبية
                   '-Y', '1.5707'], # زاوية الدوران بالعرض (90 درجة)
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        rsp,
        truck_state_publisher,
        spawn_entity,
        spawn_truck
    ])