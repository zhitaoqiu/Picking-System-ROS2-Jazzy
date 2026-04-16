import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_name = 'picking_system_core'

    # 1. 定义组件容器 (Component Container)
    # 将视觉和机械臂节点放在同一个进程中，共享内存空间
    container = ComposableNodeContainer(
        name='picking_hardware_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 加载相机生命周期组件
            ComposableNode(
                package=pkg_name,
                plugin='picking_system_core::CameraLifecycleNode',
                name='camera_node',
                extra_arguments=[{'use_intra_process_comms': True}] # 开启零拷贝优化！
            ),
            # 加载机械臂控制组件
            ComposableNode(
                package=pkg_name,
                plugin='picking_system_core::RobotControlNode',
                name='robot_control_node',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    # 2. 启动行为树主逻辑引擎
    bt_executor_node = Node(
        package=pkg_name,
        executable='bt_main_executor',
        name='bt_main_executor',
        output='screen'
    )
    
    # 3. 启动视觉处理节点（独立进程）
    vision_processor_node = Node(
        package=pkg_name,
        executable='vision_processor',
        name='vision_processor',
        output='screen'
    )
    
    # 4. 延迟启动行为树，给服务和视觉节点预热时间
    delayed_bt_executor = TimerAction(
        period=2.0,
        actions=[bt_executor_node]
    )

    return LaunchDescription([
        container,
        vision_processor_node,
        delayed_bt_executor
    ])