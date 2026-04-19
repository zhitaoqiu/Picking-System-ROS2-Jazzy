#!/usr/bin/env python3
"""
picking_system.launch.py
在 mujoco_system 基础上额外启动行为树主逻辑
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('picking_system_core')

    # 复用 mujoco_system launch
    mujoco_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mujoco_system.launch.py')
        ),
        launch_arguments={'use_viewer': 'true'}.items()
    )

    # 行为树主执行器（延迟 3 秒启动，等仿真和控制器热身）
    bt_executor = Node(
        package='picking_system_core',
        executable='bt_main_executor',
        name='bt_main_executor',
        output='screen',
    )

    delayed_bt = TimerAction(period=3.0, actions=[bt_executor])

    return LaunchDescription([
        mujoco_system,
        delayed_bt,
    ])