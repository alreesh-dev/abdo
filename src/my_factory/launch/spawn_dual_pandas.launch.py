import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')
    
    # 1. قراءة ملف URDF الجديد الذي يجمع الذراعين
    xacro_file = os.path.join(pkg_path, 'urdf', 'dual_pandas.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    # 2. تهيئة MoveIt بالملفات الجديدة (شاملة الكينماتيكا والـ SRDF والـ Controllers)
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda', package_name='moveit_resources_panda_moveit_config')
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path=os.path.join(pkg_path, 'config', 'dual_pandas.srdf'))
        # 🟢 إضافة ملف الكينماتيكا لحل مشكلة التخطيط والماوس
        .robot_description_kinematics(file_path=os.path.join(pkg_path, 'config', 'dual_kinematics.yaml'))
        .trajectory_execution(file_path=os.path.join(pkg_path, 'config', 'dual_moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='panda',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/panda/robot_description', '-name', 'dual_panda', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
    )

    # 3. تشغيل جميع المتحكمات (Controllers) للذراعين
    jsb_spawner = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '-c', '/controller_manager'])
    
    load_arm_spawner = Node(package='controller_manager', executable='spawner', arguments=['load_arm_controller', '-c', '/controller_manager'])
    load_hand_spawner = Node(package='controller_manager', executable='spawner', arguments=['load_hand_controller', '-c', '/controller_manager'])
    
    unload_arm_spawner = Node(package='controller_manager', executable='spawner', arguments=['unload_arm_controller', '-c', '/controller_manager'])
    unload_hand_spawner = Node(package='controller_manager', executable='spawner', arguments=['unload_hand_controller', '-c', '/controller_manager'])

    # 🟢 4. قراءة ملف OMPL المخصص للذراعين وحقنه في MoveGroup
    ompl_file = os.path.join(pkg_path, 'config', 'dual_ompl_planning.yaml')
    with open(ompl_file, 'r') as file:
        ompl_dict = {'ompl': yaml.safe_load(file)}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        # 👈 مررنا ompl_dict هنا ليتعرف MoveIt على الذراعين
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}, ompl_dict],
        remappings=[('/joint_states', '/panda/joint_states'), ('/robot_description', '/panda/robot_description')]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    )

    return LaunchDescription([
        rsp, spawn, static_tf, move_group,
        TimerAction(period=4.0, actions=[jsb_spawner]),
        TimerAction(period=6.0, actions=[load_arm_spawner, load_hand_spawner]),
        TimerAction(period=8.0, actions=[unload_arm_spawner, unload_hand_spawner]),
    ])