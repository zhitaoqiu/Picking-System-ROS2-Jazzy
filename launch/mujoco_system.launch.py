#!/usr/bin/env python3
"""
mujoco_system.launch.py
启动完整的 MuJoCo + Pinocchio 控制栈：
  1. mujoco_bridge        - MuJoCo 仿真桥接
  2. pinocchio_controller - FK/IK/导纳控制
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('picking_system_core')

    # 场景 XML 和 URDF 路径
    scene_xml = os.path.join(
        pkg_share, 'simulation_assets', 'franka_emika_panda', 'scene_peg_in_hole.xml'
    )
    urdf_path = os.path.join(
        pkg_share, 'simulation_assets', 'robots', 'panda_arm.urdf'
    )

    # ---------- 参数声明（允许命令行覆盖） ----------
    declare_use_viewer = DeclareLaunchArgument(
        'use_viewer', default_value='false',
        description='是否启动 MuJoCo 可视化窗口'
    )
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate', default_value='500.0',
        description='仿真发布频率 Hz'
    )

    # ---------- MuJoCo 桥接节点 ----------
    mujoco_bridge_node = Node(
        package='picking_system_core',
        executable='mujoco_bridge.py',
        name='mujoco_bridge',
        output='screen',
        parameters=[{
            'scene_xml': scene_xml,
            'use_viewer': LaunchConfiguration('use_viewer'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'control_decimation': 5,
        }]
    )

    # ---------- Pinocchio 控制器节点 ----------
    pinocchio_ctrl_node = Node(
        package='picking_system_core',
        executable='pinocchio_controller.py',
        name='pinocchio_controller',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'ee_frame_name': 'panda_link8',
            'control_rate': 500.0,
            'dls_damping': 0.03,
            'kp_pos': 2.0,
            'kp_ori': 2.0,
        }]
    )

    return LaunchDescription([
        declare_use_viewer,
        declare_publish_rate,
        mujoco_bridge_node,
        pinocchio_ctrl_node,
    ])