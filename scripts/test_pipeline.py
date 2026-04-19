#!/usr/bin/env python3
"""
test_pipeline.py
验证完整通信管线是否打通：
  1. 发布 /franka/controller_enable = True
  2. 发布一个目标位姿到 /franka/target_pose
  3. 监听 /franka/ee_pose 和 /franka/joint_states，打印接收情况

用法（新终端里运行，需要先 launch mujoco_system）：
  python3 scripts/test_pipeline.py
"""

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class PipelineTester(Node):

    def __init__(self):
        super().__init__('pipeline_tester')

        self.target_pub = self.create_publisher(PoseStamped, '/franka/target_pose', 10)
        self.enable_pub = self.create_publisher(Bool, '/franka/controller_enable', 10)

        self.create_subscription(JointState, '/franka/joint_states', self._js_cb, 10)
        self.create_subscription(PoseStamped, '/franka/ee_pose', self._ee_cb, 10)

        self._js_count = 0
        self._ee_count = 0

        # 等 1 秒再发指令，给节点启动时间
        self.create_timer(1.0, self._send_commands)
        self.create_timer(2.0, self._print_status)

    def _send_commands(self):
        # 启用控制器
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)

        # 发布目标位姿（机械臂正前方一个安全点）
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'panda_link0'
        pose_msg.pose.position.x = 0.5
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.35   # 较高的安全位置
        pose_msg.pose.orientation.w = 1.0  # 单位四元数，不旋转
        self.target_pub.publish(pose_msg)

        self.get_logger().info('已发送：controller_enable=True，target_pose=[0.5, 0.0, 0.4]')

    def _js_cb(self, msg):
        self._js_count += 1
        if self._js_count % 50 == 1:
            pos = np.round(msg.position[:7], 3).tolist()
            self.get_logger().info(f'[joint_states #{self._js_count}] q={pos}')

    def _ee_cb(self, msg):
        self._ee_count += 1
        if self._ee_count % 50 == 1:
            p = msg.pose.position
            self.get_logger().info(
                f'[ee_pose #{self._ee_count}] '
                f'pos=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]'
            )

    def _print_status(self):
        self.get_logger().info(
            f'状态统计 | joint_states收到:{self._js_count} | ee_pose收到:{self._ee_count}'
        )
        if self._js_count > 0 and self._ee_count > 0:
            self.get_logger().info('✅ 通信管线打通！')
        else:
            self.get_logger().warn('❌ 管线未打通，检查节点是否正常启动')


def main():
    rclpy.init()
    node = PipelineTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()