import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_path = get_package_share_directory('my_factory')
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'panda_ign.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda', package_name='moveit_resources_panda_moveit_config')
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path='config/panda.srdf')
        .trajectory_execution(file_path=os.path.join(pkg_path, 'config', 'moveit_controllers.yaml'))
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
        # 🟢 تصفير الإحداثيات هنا لمنع الإزاحة المضاعفة في جازيبو
        arguments=['-topic', '/panda/robot_description', '-name', 'panda_arm', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
    )

    # 🟢 تأخير تشغيل المشغلات برمجياً لضمان اكتمال تحميل جازيبو وربطها بالمدير
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['panda_arm_controller', '-c', '/controller_manager'],
        output='screen'
    )

    gripper_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['panda_hand_controller', '-c', '/controller_manager'],
        output='screen'
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        remappings=[
            ('/joint_states', '/panda/joint_states'),
            ('/robot_description', '/panda/robot_description')
        ]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # 🟢 ربط الأرضيات ببعضها من نقطة الصفر، وستأخذ الذراع موقعها الدقيق من الـ URDF
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        spawn,
        static_tf,
        move_group,
        
        # 🟢 استخدام المؤقت الزمني (Timer) بدلاً من انتظار خروج العقدة لمنع التعليق
        TimerAction(period=4.0, actions=[jsb_spawner]),
        TimerAction(period=6.0, actions=[arm_spawner]),
        TimerAction(period=8.0, actions=[gripper_spawner]),
    ])