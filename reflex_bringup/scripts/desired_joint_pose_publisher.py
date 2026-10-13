#!/usr/bin/env python3
"""
Node that reads the current joint position from /joint_states at startup,
then publishes a circular motion in joint space (oscillating joints 1 and 3)
to the franka_joint_impedance_controller.

Usage (from the ROS2 container):
    python3 desired_joint_pose_publisher.py

The node publishes a JointGoal message to:
    /panda/franka_joint_impedance_controller/desired_pose

You can also adjust the stiffness scale at runtime via:
    ros2 service call /panda/franka_joint_impedance_controller/parameters \
        multi_mode_control_msgs/srv/SetJointImpedance "{stiffness_scale: 0.5}"
"""

import math
import rclpy
from rclpy.node import Node
from multi_mode_control_msgs.msg import JointGoal
from sensor_msgs.msg import JointState


class DesiredJointPosePublisher(Node):
    def __init__(self):
        super().__init__('desired_joint_pose_publisher')

        # Publisher on the joint impedance controller desired_pose topic
        self.publisher_ = self.create_publisher(
            JointGoal,
            '/panda/franka_joint_impedance_controller/desired_pose',
            10
        )

        # Circular motion parameters
        self.amplitude = 0.3     # rad – keep conservative for safety
        self.omega = 1        # rad/s – slow circular motion
        self.joint_a = 1         # first oscillating joint index  (joint 2 / shoulder)
        self.joint_b = 3         # second oscillating joint index (joint 4 / elbow)

        # Time tracking
        self.t = 0.0
        self.dt = 0.02  # 50 Hz

        # Will be filled once first /joint_states message arrives
        self.q_start = None

        # Subscribe to /joint_states to capture the starting position
        self.get_logger().info('Waiting for first /joint_states message...')
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

    def _joint_state_callback(self, msg: JointState):
        """Capture the first joint_states message as start position, then unsubscribe."""
        if self.q_start is not None:
            return  # already initialized

        # joint_states may contain gripper joints too — extract only the 7 arm joints
        # Panda joint names: panda_joint1 .. panda_joint7
        arm_joint_names = [f'panda_joint{i}' for i in range(1, 8)]

        q = [0.0] * 7
        for i, name in enumerate(arm_joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                q[i] = msg.position[idx]
            else:
                self.get_logger().warn(f'{name} not found in /joint_states, using 0.0')

        self.q_start = q
        self.get_logger().info(f'Start position: {[round(v, 3) for v in self.q_start]}')
        self.get_logger().info(
            f'Circular motion: joints [{self.joint_a}] and [{self.joint_b}], '
            f'amplitude={self.amplitude} rad, omega={self.omega} rad/s'
        )

        # Unsubscribe — no longer needed
        self.destroy_subscription(self.joint_state_sub)

        # Start the publishing timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

    # ── Publishing loop ───────────────────────────────────────────────
    def timer_callback(self):
        msg = JointGoal()

        q_target = list(self.q_start)

        # Circular motion: joint_a follows sin, joint_b follows cos
        # This traces a circle in the (joint_a, joint_b) plane
        q_target[self.joint_a] += self.amplitude * math.sin(self.omega * self.t)
        q_target[self.joint_b] += self.amplitude * math.cos(self.omega * self.t)

        msg.q = q_target
        msg.qd = [0.0] * 7
        self.publisher_.publish(msg)

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = DesiredJointPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down desired_joint_pose_publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
