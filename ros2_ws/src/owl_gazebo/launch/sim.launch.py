import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = get_package_share_directory('owl_gazebo')
    xacro_file = os.path.join(pkg_path, 'urdf', 'sim_gazebo.xacro')
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    custom_world_path = os.path.join(pkg_path, 'world', 'table.world')
    gazebo_launcher = ExecuteProcess(
            cmd=['gazebo', '--verbose',custom_world_path,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='gripper',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-entity', 'gripper',
                   '-x', '-0.4',   
                    '-y', '0.0',
                    '-z', '1.07']
    )
    
    image_processing_node = Node(
        package='owl_cv',
        executable='image_processing_node',
        name='image_processing_node',
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity,
        gazebo_launcher,
        image_processing_node
    ])