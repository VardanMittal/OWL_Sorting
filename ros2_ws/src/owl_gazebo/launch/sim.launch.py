import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Declare use_sim_time
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get directories
    pkg_gazebo = get_package_share_directory('owl_gazebo')
    pkg_moveit_config = get_package_share_directory('owl_moveit_config')

    # Process URDF file
    xacro_file = os.path.join(pkg_gazebo, 'urdf', 'sim_gazebo.xacro')
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(pkg_gazebo, 'world', 'table.world')}.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'owl',
                   '-x', '-0.4',   
                   '-y', '0.0',
                   '-z', '1.07'],
        output='screen',
    )

    # Load controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    # MoveIt 2 launch
    moveit_config = os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_config),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        load_joint_state_controller,
        load_arm_controller,
        moveit
    ])