#!/usr/bin/env python3
"""
Pinocchio 控制器节点
职责：
  1. 订阅 /franka/joint_states，维护机器人当前运动学状态
  2. 订阅 /franka/ee_wrench，获取末端力觉
  3. 订阅 /franka/target_pose，接收上层（BT）发来的末端目标位姿
  4. 用 Pinocchio 做 FK / Jacobian / DLS IK
  5. 用导纳控制器做接触柔顺
  6. 发布 /franka/joint_command 给 MuJoCo 桥接节点执行

话题汇总：
  订阅:
    /franka/joint_states     (sensor_msgs/JointState)
    /franka/ee_wrench        (geometry_msgs/WrenchStamped)
    /franka/target_pose      (geometry_msgs/PoseStamped)
  发布:
    /franka/joint_command    (sensor_msgs/JointState)
    /franka/ee_pose          (geometry_msgs/PoseStamped)   当前末端位姿
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import pinocchio as pin
import threading
import os

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Bool


class AdmittanceController:
    """笛卡尔空间导纳控制器，与项目1保持一致"""

    def __init__(self, mass=1.0, damping=20.0, stiffness=50.0, dt=0.002):
        self.dt = dt
        self.M_inv = np.eye(6) / mass
        self.D = damping * np.eye(6)
        self.K = stiffness * np.eye(6)
        self.x_c = np.zeros(6)
        self.dx_c = np.zeros(6)
        self.is_initialized = False

    def reset(self, current_pose_6d: np.ndarray):
        self.x_c = current_pose_6d.copy()
        self.dx_c = np.zeros(6)
        self.is_initialized = True

    def step(self, x_target: np.ndarray, current_pose: np.ndarray, f_ext: np.ndarray) -> np.ndarray:
        if not self.is_initialized:
            self.x_c = current_pose.copy()
            self.is_initialized = True

        f_spring = self.K @ (x_target - self.x_c)
        f_damper = self.D @ (-self.dx_c)
        ddx_c = self.M_inv @ (f_ext + f_spring + f_damper)

        self.dx_c += ddx_c * self.dt
        self.x_c += self.dx_c * self.dt

        return self.x_c.copy()


class PinocchioController(Node):

    def __init__(self):
        super().__init__('pinocchio_controller')

        # ============================================================
        # 1. 参数
        # ============================================================
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('ee_frame_name', 'panda_link8')
        self.declare_parameter('control_rate', 500.0)   # Hz
        self.declare_parameter('dls_damping', 0.03)
        self.declare_parameter('kp_pos', 2.0)
        self.declare_parameter('kp_ori', 2.0)

        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        ee_frame = self.get_parameter('ee_frame_name').get_parameter_value().string_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.damping = self.get_parameter('dls_damping').get_parameter_value().double_value
        self.kp_pos = self.get_parameter('kp_pos').get_parameter_value().double_value
        self.kp_ori = self.get_parameter('kp_ori').get_parameter_value().double_value

        # ============================================================
        # 2. Pinocchio 模型加载
        # ============================================================
        if not urdf_path or not os.path.exists(urdf_path):
            self.get_logger().error(f'URDF 文件不存在: "{urdf_path}"')
            raise RuntimeError(f'无效的 urdf_path: {urdf_path}')

        self.get_logger().info(f'加载 URDF: {urdf_path}')
        self.pin_model = pin.buildModelFromUrdf(urdf_path)
        self.pin_data = self.pin_model.createData()

        if not self.pin_model.existFrame(ee_frame):
            raise RuntimeError(f'URDF 中不存在 frame: {ee_frame}')
        self.ee_frame_id = self.pin_model.getFrameId(ee_frame)

        self.get_logger().info(
            f'Pinocchio 模型加载成功 | nq={self.pin_model.nq} | ee_frame={ee_frame}'
        )

        # ============================================================
        # 3. 内部状态
        # ============================================================
        self._lock = threading.Lock()
        self._dt = 1.0 / control_rate

        # 当前关节状态
        self._q = np.zeros(self.pin_model.nq)
        self._dq = np.zeros(self.pin_model.nv)
        self._has_joint_state = False

        # 当前末端力觉
        self._f_ext = np.zeros(6)

        # 目标位姿（来自BT）
        self._target_pos = None
        self._target_rot = None
        self._is_active = True  # 默认启动即激活，BT 可通过 /franka/controller_enable 动态控制

        # 导纳控制器
        self._admittance = AdmittanceController(
            mass=1.0, damping=20.0, stiffness=50.0, dt=self._dt
        )

        # ============================================================
        # 4. ROS2 通信
        # ============================================================
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅
        self.create_subscription(
            JointState, '/franka/joint_states',
            self._joint_state_callback, reliable_qos
        )
        self.create_subscription(
            WrenchStamped, '/franka/ee_wrench',
            self._wrench_callback, reliable_qos
        )
        self.create_subscription(
            PoseStamped, '/franka/target_pose',
            self._target_pose_callback, reliable_qos
        )
        self.create_subscription(
            Bool, '/franka/controller_enable',
            self._enable_callback, reliable_qos
        )

        # 发布
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/franka/joint_command', reliable_qos
        )
        self.ee_pose_pub = self.create_publisher(
            PoseStamped, '/franka/ee_pose', reliable_qos
        )

        # 控制主循环
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'Pinocchio 控制器就绪 | 控制频率={control_rate}Hz'
        )

    # ============================================================
    # 订阅回调
    # ============================================================
    def _joint_state_callback(self, msg: JointState):
        if len(msg.position) < 7:
            return
        with self._lock:
            self._q[:7] = np.array(msg.position[:7])
            if len(msg.velocity) >= 7:
                self._dq[:7] = np.array(msg.velocity[:7])
            self._has_joint_state = True

    def _wrench_callback(self, msg: WrenchStamped):
        with self._lock:
            self._f_ext = np.array([
                msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
            ])

    def _target_pose_callback(self, msg: PoseStamped):
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        quat = np.array([
            msg.pose.orientation.w,  # pinocchio 的四元数顺序是 wxyz
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ])
        rot = pin.Quaternion(quat).toRotationMatrix()

        with self._lock:
            self._target_pos = pos
            self._target_rot = rot
            self._admittance.is_initialized = False  # 目标变化时重置导纳积分

        self.get_logger().info(
            f'收到目标位姿: pos={np.round(pos, 3)}'
        )

    def _enable_callback(self, msg: Bool):
        with self._lock:
            self._is_active = msg.data
        self.get_logger().info(f'控制器{"启用" if msg.data else "停用"}')

    # ============================================================
    # 控制主循环
    # ============================================================
    def _control_loop(self):
        with self._lock:
            if not self._has_joint_state or not self._is_active:
                return
            if self._target_pos is None:
                return

            q = self._q.copy()
            dq = self._dq.copy()
            target_pos = self._target_pos.copy()
            target_rot = self._target_rot.copy()
            f_ext = self._f_ext.copy()

        # ---------- FK ----------
        pin.computeAllTerms(self.pin_model, self.pin_data, q, dq)
        pin.updateFramePlacements(self.pin_model, self.pin_data)

        ee = self.pin_data.oMf[self.ee_frame_id]
        curr_pos = ee.translation.copy()
        curr_rot = ee.rotation.copy()

        # ---------- 发布当前末端位姿 ----------
        self._publish_ee_pose(curr_pos, curr_rot)

        # ---------- 导纳控制 ----------
        curr_pose_6d = np.concatenate([curr_pos, pin.log3(curr_rot)])
        target_pose_6d = np.concatenate([target_pos, pin.log3(target_rot)])

        compliant_pose = self._admittance.step(target_pose_6d, curr_pose_6d, f_ext)

        # ---------- DLS IK ----------
        joint_vel = self._dls_ik(q, dq, compliant_pose[:3], target_rot)

        # ---------- 积分得关节位置目标 ----------
        q_target = q[:7] + joint_vel * self._dt

        # 关节限位裁剪
        q_min = self.pin_model.lowerPositionLimit[:7]
        q_max = self.pin_model.upperPositionLimit[:7]
        q_target = np.clip(q_target, q_min, q_max)

        # ---------- 发布关节指令 ----------
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = [f'joint{i+1}' for i in range(7)]
        cmd.position = q_target.tolist()
        self.joint_cmd_pub.publish(cmd)

    # ============================================================
    # DLS 逆运动学（与项目1完全一致）
    # ============================================================
    def _dls_ik(self, q, dq, target_pos, target_rot) -> np.ndarray:
        curr_pos = self.pin_data.oMf[self.ee_frame_id].translation
        curr_rot = self.pin_data.oMf[self.ee_frame_id].rotation

        pos_err = target_pos - curr_pos

        R_err = curr_rot.T @ target_rot
        ori_err_local = pin.log3(R_err)
        ori_err_world = curr_rot @ ori_err_local

        v_d = np.zeros(6)
        v_d[:3] = self.kp_pos * pos_err
        v_d[3:] = self.kp_ori * ori_err_world

        J = pin.getFrameJacobian(
            self.pin_model, self.pin_data,
            self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )

        lambda_sq = self.damping ** 2
        JJt = J @ J.T
        joint_vel = J.T @ np.linalg.inv(JJt + lambda_sq * np.eye(6)) @ v_d

        return joint_vel[:7]

    # ============================================================
    # 发布末端位姿
    # ============================================================
    def _publish_ee_pose(self, pos: np.ndarray, rot: np.ndarray):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_link0'

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        quat = pin.Quaternion(rot)
        msg.pose.orientation.w = float(quat.w)
        msg.pose.orientation.x = float(quat.x)
        msg.pose.orientation.y = float(quat.y)
        msg.pose.orientation.z = float(quat.z)

        self.ee_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PinocchioController()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[PinocchioController] 启动失败: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()