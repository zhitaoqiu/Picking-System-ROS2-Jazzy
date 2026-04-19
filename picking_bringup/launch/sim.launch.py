from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    description_share = get_package_share_directory('picking_description')
    bringup_share = get_package_share_directory('picking_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    xacro_file = os.path.join(description_share, 'urdf', 'panda_arm.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'mujoco_model': os.path.join(description_share, 'mjcf', 'scene_peg_in_hole.xml'),
            'headless': 'false',
            'pids_file': os.path.join(bringup_share, 'config', 'pids.yaml'),
        },
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    controllers_yaml = os.path.join(bringup_share, 'config', 'controllers.yaml')

    control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            controllers_yaml,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    ft_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_sensor_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    admittance_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['admittance_controller', '--inactive', '-c', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        ft_broadcaster,
        velocity_controller,
        admittance_controller,
    ])
