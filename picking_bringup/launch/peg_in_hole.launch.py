from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('picking_bringup')

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'servo.launch.py')
        )
    )

    pinocchio_node = Node(
        package='picking_pinocchio_solver',
        executable='pinocchio_solver_node.py',
        output='screen',
        parameters=[
            {
                'urdf_path': os.path.join(
                    get_package_share_directory('picking_description'),
                    'urdf',
                    'panda_arm.urdf'
                ),
                'ee_frame': 'panda_link8',
                'publish_rate_hz': 50.0,
            }
        ],
    )

    bt_executor = Node(
        package='picking_task_bt',
        executable='bt_main_executor',
        output='screen',
        parameters=[
            {
                'tree_file': os.path.join(
                    get_package_share_directory('picking_task_bt'),
                    'behavior_trees',
                    'peg_in_hole_servo.xml'
                )
            }
        ],
    )

    return LaunchDescription([
        servo_launch,
        pinocchio_node,
        bt_executor,
    ])
