#!/usr/bin/env python3
import json
from typing import Optional

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class PinocchioSolverNode(Node):
    def __init__(self) -> None:
        super().__init__('pinocchio_solver_node')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('ee_frame', 'panda_link8')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('ik_damping', 1e-3)
        self.declare_parameter('ik_step', 0.3)
        self.declare_parameter('ik_max_iterations', 80)

        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.ik_damping = self.get_parameter('ik_damping').get_parameter_value().double_value
        self.ik_step = self.get_parameter('ik_step').get_parameter_value().double_value
        self.ik_max_iterations = self.get_parameter('ik_max_iterations').get_parameter_value().integer_value

        if not urdf_path:
            raise RuntimeError('pinocchio_solver_node requires parameter urdf_path')

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(self.ee_frame)
        if self.frame_id >= len(self.model.frames):
            raise RuntimeError(f'Frame {self.ee_frame} not found in model')

        self.joint_names = [
            name for name in self.model.names if name.startswith('panda_joint')
        ]
        self.current_q = np.zeros(self.model.nq)
        self.current_v = np.zeros(self.model.nv)
        self.have_state = False
        self.last_goal: Optional[PoseStamped] = None

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/pinocchio/goal_pose', self.on_goal_pose, 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, '/pinocchio/ee_pose', 10)
        self.ik_solution_pub = self.create_publisher(JointState, '/pinocchio/ik_solution', 10)
        self.debug_pub = self.create_publisher(String, '/pinocchio/debug', 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info('Pinocchio solver node started in diagnostics-only mode')

    def on_joint_state(self, msg: JointState) -> None:
        joint_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        joint_vel = {name: vel for name, vel in zip(msg.name, msg.velocity)} if msg.velocity else {}

        q = np.zeros(self.model.nq)
        v = np.zeros(self.model.nv)
        for idx, name in enumerate(self.joint_names, start=1):
            if name in joint_pos:
                q[idx - 1] = joint_pos[name]
            if name in joint_vel:
                v[idx - 1] = joint_vel[name]

        self.current_q = q
        self.current_v = v
        self.have_state = True

    def on_goal_pose(self, msg: PoseStamped) -> None:
        self.last_goal = msg
        if not self.have_state:
            return

        q_sol = self.solve_dls_ik(msg)
        if q_sol is None:
            self.get_logger().warn('Pinocchio IK did not converge')
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        out.position = q_sol.tolist()
        self.ik_solution_pub.publish(out)

    def solve_dls_ik(self, goal: PoseStamped) -> Optional[np.ndarray]:
        target = pin.SE3(
            pin.Quaternion(
                goal.pose.orientation.w,
                goal.pose.orientation.x,
                goal.pose.orientation.y,
                goal.pose.orientation.z,
            ).matrix(),
            np.array([
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
            ]),
        )

        q = self.current_q.copy()
        for _ in range(self.ik_max_iterations):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current = self.data.oMf[self.frame_id]
            error_se3 = current.actInv(target)
            error = pin.log6(error_se3).vector
            if np.linalg.norm(error) < 1e-4:
                return q[: len(self.joint_names)]

            j = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id, pin.ReferenceFrame.LOCAL)
            damping_matrix = self.ik_damping * np.eye(6)
            dq = j.T @ np.linalg.solve(j @ j.T + damping_matrix, error)
            q = pin.integrate(self.model, q, self.ik_step * dq)

        return None

    def on_timer(self) -> None:
        if not self.have_state:
            return

        pin.computeAllTerms(self.model, self.data, self.current_q, self.current_v)
        pin.forwardKinematics(self.model, self.data, self.current_q, self.current_v)
        pin.updateFramePlacements(self.model, self.data)
        pose = self.data.oMf[self.frame_id]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(pose.translation[0])
        pose_msg.pose.position.y = float(pose.translation[1])
        pose_msg.pose.position.z = float(pose.translation[2])
        quat = pin.Quaternion(pose.rotation)
        pose_msg.pose.orientation.x = float(quat.x)
        pose_msg.pose.orientation.y = float(quat.y)
        pose_msg.pose.orientation.z = float(quat.z)
        pose_msg.pose.orientation.w = float(quat.w)
        self.ee_pose_pub.publish(pose_msg)

        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            self.current_q,
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        gravity = pin.computeGeneralizedGravity(self.model, self.data, self.current_q)
        nle = pin.nonLinearEffects(self.model, self.data, self.current_q, self.current_v)

        debug = {
            'frame': self.ee_frame,
            'gravity': gravity[: len(self.joint_names)].tolist(),
            'nonlinear_effects': nle[: len(self.joint_names)].tolist(),
            'jacobian_first_row': jacobian[0, : len(self.joint_names)].tolist(),
        }
        msg = String()
        msg.data = json.dumps(debug)
        self.debug_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PinocchioSolverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
