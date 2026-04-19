from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml


def generate_launch_description():
    description_share = get_package_share_directory('picking_description')
    bringup_share = get_package_share_directory('picking_bringup')
    moveit_share = get_package_share_directory('picking_moveit_config')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim.launch.py')
        )
    )

    robot_description = {
        'robot_description': xacro.process_file(
            os.path.join(description_share, 'urdf', 'panda_arm.urdf.xacro'),
            mappings={
                'mujoco_model': os.path.join(description_share, 'mjcf', 'scene_peg_in_hole.xml'),
                'headless': 'false',
                'pids_file': os.path.join(bringup_share, 'config', 'pids.yaml'),
            },
        ).toxml()
    }

    with open(os.path.join(moveit_share, 'srdf', 'panda.srdf'), 'r', encoding='utf-8') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    with open(os.path.join(moveit_share, 'config', 'kinematics.yaml'), 'r', encoding='utf-8') as f:
        robot_description_kinematics = {'robot_description_kinematics': yaml.safe_load(f)}

    with open(os.path.join(moveit_share, 'config', 'moveit_servo.yaml'), 'r', encoding='utf-8') as f:
        servo_params = yaml.safe_load(f)

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_params,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        sim_launch,
        servo_node,
    ])
