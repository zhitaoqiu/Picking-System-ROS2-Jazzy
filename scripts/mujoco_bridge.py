#!/usr/bin/env python3
"""
MuJoCo-ROS2 桥接节点
职责：
  1. 加载 MuJoCo 场景
  2. 订阅 /franka/joint_command，写入 MuJoCo ctrl
  3. 每个控制周期推进仿真 mj_step()
  4. 发布 /franka/joint_states（关节状态）
  5. 发布 /franka/ee_wrench（末端力/力矩，来自传感器）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import mujoco
import numpy as np
import threading
import os

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped


# 关节名称，与 panda.xml 里的 joint name 对应
PANDA_JOINT_NAMES = [
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7"
]


class MujocoBridge(Node):

    def __init__(self):
        super().__init__('mujoco_bridge')

        # ============================================================
        # 1. 参数声明
        # ============================================================
        self.declare_parameter('scene_xml', '')
        self.declare_parameter('use_viewer', False)
        self.declare_parameter('publish_rate', 500.0)   # Hz，仿真推进频率
        self.declare_parameter('control_decimation', 5) # 每几个仿真步响应一次控制指令

        scene_xml = self.get_parameter('scene_xml').get_parameter_value().string_value
        self.use_viewer = self.get_parameter('use_viewer').get_parameter_value().bool_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.control_decimation = self.get_parameter('control_decimation').get_parameter_value().integer_value

        # ============================================================
        # 2. 加载 MuJoCo 模型
        # ============================================================
        if not scene_xml or not os.path.exists(scene_xml):
            self.get_logger().error(f'场景文件不存在或未指定: "{scene_xml}"')
            raise RuntimeError(f'无效的 scene_xml 路径: {scene_xml}')

        self.get_logger().info(f'加载 MuJoCo 场景: {scene_xml}')
        self.mj_model = mujoco.MjModel.from_xml_path(scene_xml)
        self.mj_data = mujoco.MjData(self.mj_model)

        # 内部状态锁（仿真线程与 ROS 回调线程共享数据）
        self._lock = threading.Lock()

        # 初始化仿真到 home 姿态
        self._reset_to_home()

        # 力传感器去皮基准（在 home 姿态下标定）
        self._ft_bias = self._read_raw_wrench()

        self.get_logger().info(
            f'MuJoCo 模型加载成功 | '
            f'nq={self.mj_model.nq} nv={self.mj_model.nv} nu={self.mj_model.nu}'
        )

        # ============================================================
        # 3. ROS2 通信接口
        # ============================================================
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅：接收上层控制器发来的关节位置指令
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/franka/joint_command',
            self._joint_command_callback,
            reliable_qos
        )

        # 发布：关节状态（位置、速度）
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/franka/joint_states',
            reliable_qos
        )

        # 发布：末端力觉（已去皮）
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/franka/ee_wrench',
            reliable_qos
        )

        # 仿真主循环定时器
        sim_period = 1.0 / publish_rate
        self.sim_timer = self.create_timer(sim_period, self._sim_step_callback)

        # ============================================================
        # 4. 可选：启动 MuJoCo 可视化窗口
        # ============================================================
        self._viewer = None
        if self.use_viewer:
            # viewer 需要在主线程运行，这里用被动模式
            import mujoco.viewer as mjviewer
            self._viewer = mjviewer.launch_passive(self.mj_model, self.mj_data)
            self.get_logger().info('MuJoCo Viewer 已启动')

        self.get_logger().info(
            f'MuJoCo Bridge 就绪 | '
            f'仿真频率={publish_rate}Hz | '
            f'控制降频={self.control_decimation}'
        )

    # ============================================================
    # 核心：仿真主循环
    # ============================================================
    def _sim_step_callback(self):
        with self._lock:
            # 推进物理仿真一步
            mujoco.mj_step(self.mj_model, self.mj_data)

        # 发布当前状态
        self._publish_joint_states()
        self._publish_ee_wrench()

        # 同步可视化窗口
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    # ============================================================
    # 控制指令回调：接收关节位置目标
    # ============================================================
    def _joint_command_callback(self, msg: JointState):
        if len(msg.position) < 7:
            self.get_logger().warn(
                f'收到关节指令维度不足: {len(msg.position)} < 7，忽略'
            )
            return

        with self._lock:
            # 写入前7个关节的位置控制目标
            for i in range(7):
                self.mj_data.ctrl[i] = msg.position[i]

            # 如果消息里包含夹爪指令（第8个元素）
            if len(msg.position) >= 8 and self.mj_model.nu > 7:
                self.mj_data.ctrl[7] = msg.position[7]

    # ============================================================
    # 发布关节状态
    # ============================================================
    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = PANDA_JOINT_NAMES

        with self._lock:
            msg.position = self.mj_data.qpos[:7].tolist()
            msg.velocity = self.mj_data.qvel[:7].tolist()
            # effort 用关节力矩近似（actuator_force）
            msg.effort = self.mj_data.actuator_force[:7].tolist()

        self.joint_state_pub.publish(msg)

    # ============================================================
    # 发布末端力觉
    # ============================================================
    def _publish_ee_wrench(self):
        raw = self._read_raw_wrench()
        calibrated = raw - self._ft_bias  # 去皮

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_link8'
        msg.wrench.force.x = float(calibrated[0])
        msg.wrench.force.y = float(calibrated[1])
        msg.wrench.force.z = float(calibrated[2])
        msg.wrench.torque.x = float(calibrated[3])
        msg.wrench.torque.y = float(calibrated[4])
        msg.wrench.torque.z = float(calibrated[5])

        self.wrench_pub.publish(msg)

    # ============================================================
    # 工具函数
    # ============================================================
    def _read_raw_wrench(self) -> np.ndarray:
        """读取传感器原始数据，返回 6 维向量"""
        try:
            force = self.mj_data.sensor('ee_force_sensor').data.copy()
            torque = self.mj_data.sensor('ee_torque_sensor').data.copy()
            return np.concatenate([force, torque])
        except Exception:
            return np.zeros(6)

    def _reset_to_home(self):
        """重置到 home 关节位姿"""
        mujoco.mj_resetData(self.mj_model, self.mj_data)

        home_q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.mj_data.qpos[:7] = home_q

        # 张开夹爪
        if self.mj_model.nq > 8:
            self.mj_data.qpos[7] = 0.04
            self.mj_data.qpos[8] = 0.04
        if self.mj_model.nu > 7:
            self.mj_data.ctrl[7] = 255.0

        mujoco.mj_forward(self.mj_model, self.mj_data)
        self.get_logger().info('仿真已重置到 home 姿态')

    def destroy_node(self):
        if self._viewer is not None:
            self._viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MujocoBridge()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[MujocoBridge] 启动失败: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()