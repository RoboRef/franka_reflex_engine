#!/usr/bin/env python3
"""
Task Script:
1. Move to Joint Position 1: [1.44267, 0.5971, 0.262509, -2.04938, -0.317762, 2.60155, -0.472003]
2. Move to Joint Position 2: [1.49287, 0.933638, 0.150206, -1.89913, -0.349659, 2.81977, -0.481786]
3. Switch to Cartesian Impedance Controller and move 10 cm along positive X direction.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from multi_mode_control_msgs.srv import SetControllers
from multi_mode_control_msgs.msg import Controller, JointGoal, CartesianImpedanceGoal
from tf2_ros import Buffer, TransformListener, TransformException


class MoveTask(Node):
    def __init__(self):
        super().__init__('move_task')

        # Target joint positions
        self.q1 = [1.44267, 0.5971, 0.262509, -2.04938, -0.317762, 2.60155, -0.472003]
        self.q2 = [1.34507,0.871697,0.165614,-1.99556,-0.388187,2.83158,-0.584412]

        # 1. Controller Switch Client
        self.set_ctrl_client = self.create_client(
            SetControllers,
            '/reflex_mc_controller/set_controllers'
        )

        # 2. Joint Goal Publisher
        self.joint_pub = self.create_publisher(
            JointGoal,
            '/panda/panda_joint_impedance_controller/desired_pose',
            10
        )

        # 3. Cartesian Goal Publisher
        self.cart_pub = self.create_publisher(
            CartesianImpedanceGoal,
            '/panda/panda_cartesian_impedance_controller/desired_pose',
            10
        )

        # 4. TF Listener for Cartesian initial EE pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 5. Joint State Subscriber
        self.current_q = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Execution State Machine
        self.state = "WAIT_FOR_JOINTS"
        self.q_start_motion = None
        self.start_ee_pose = None

        self.phase_duration = 2.0      # 4 seconds per joint transition
        self.cart_duration = 1.0       # 5 seconds for Cartesian +X motion
        self.cart_target_distance = 0.20  # 10 cm along +X
        self.timer_period = 0.01      # 200 Hz (5 ms period)
        self.phase_time = 0.0

        self.get_logger().info("Initializing Task (Joint P1 -> Joint P2 -> Cartesian +X)...")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def joint_callback(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.current_q = [joint_map[f'panda_joint{i}'] for i in range(1, 8)]
        except (KeyError, ValueError):
            if len(msg.position) >= 7:
                self.current_q = list(msg.position[:7])

    def switch_controller(self, controller_name, next_state):
        self.get_logger().info(f"Switching active controller to '{controller_name}'...")
        if not self.set_ctrl_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_controllers service unavailable!")
            return False

        req = SetControllers.Request()
        ctrl = Controller()
        ctrl.name = controller_name
        ctrl.resources = ["panda"]
        req.controllers = [ctrl]

        future = self.set_ctrl_client.call_async(req)
        def callback(fut):
            try:
                fut.result()
                self.get_logger().info(f"Successfully activated controller '{controller_name}'.")
                self.state = next_state
                self.phase_time = 0.0
                self.q_start_motion = list(self.current_q) if self.current_q else list(self.q1)
            except Exception as e:
                self.get_logger().error(f"Failed to switch controller: {e}")
        future.add_done_callback(callback)
        return True

    def timer_callback(self):
        if self.current_q is None:
            self.get_logger().info("Waiting for initial /joint_states...", throttle_duration_sec=1.0)
            return

        # Phase 0: Start sequence -> Switch to Joint Impedance Controller
        if self.state == "WAIT_FOR_JOINTS":
            self.state = "SWITCHING_TO_JOINT_CTRL"
            self.switch_controller("panda_joint_impedance_controller", "MOVE_TO_P1")
            return

        elif self.state == "SWITCHING_TO_JOINT_CTRL":
            # Waiting for service async callback
            return

        # Phase 1: Interpolate to Joint Position 1
        elif self.state == "MOVE_TO_P1":
            self.phase_time += self.timer_period
            progress = min(1.0, self.phase_time / self.phase_duration)
            s = 3.0 * (progress ** 2) - 2.0 * (progress ** 3)  # smoothstep

            q_goal = [
                self.q_start_motion[i] + s * (self.q1[i] - self.q_start_motion[i])
                for i in range(7)
            ]

            msg = JointGoal()
            msg.q = q_goal
            msg.qd = [0.0] * 7
            self.joint_pub.publish(msg)

            self.get_logger().info(
                f"Phase 1: Moving to Joint P1 ({progress*100:.0f}%)",
                throttle_duration_sec=1.0
            )

            if progress >= 1.0:
                self.get_logger().info("Reached Joint Position 1!")
                self.state = "PREPARE_MOVE_TO_P2"
                self.phase_time = 0.0

        elif self.state == "PREPARE_MOVE_TO_P2":
            self.phase_time += self.timer_period
            if self.phase_time >= 0.5:  # small pause
                self.state = "MOVE_TO_P2"
                self.q_start_motion = list(self.q1)
                self.phase_time = 0.0

        # Phase 2: Interpolate from Joint Position 1 to Joint Position 2
        elif self.state == "MOVE_TO_P2":
            self.phase_time += self.timer_period
            progress = min(1.0, self.phase_time / self.phase_duration)
            s = 3.0 * (progress ** 2) - 2.0 * (progress ** 3)  # smoothstep

            q_goal = [
                self.q1[i] + s * (self.q2[i] - self.q1[i])
                for i in range(7)
            ]

            msg = JointGoal()
            msg.q = q_goal
            msg.qd = [0.0] * 7
            self.joint_pub.publish(msg)

            self.get_logger().info(
                f"Phase 2: Moving to Joint P2 ({progress*100:.0f}%)",
                throttle_duration_sec=1.0
            )

            if progress >= 1.0:
                self.get_logger().info("Reached Joint Position 2!")
                self.state = "PREPARE_SWITCH_TO_CART"
                self.phase_time = 0.0

        elif self.state == "PREPARE_SWITCH_TO_CART":
            self.phase_time += self.timer_period
            if self.phase_time >= 0.5:
                self.state = "SWITCHING_TO_CART_CTRL"
                self.switch_controller("panda_cartesian_impedance_controller", "CAPTURE_CART_START")

        elif self.state == "SWITCHING_TO_CART_CTRL":
            # Waiting for service async callback
            return

        # Phase 3: Capture starting Cartesian EE pose from TF
        elif self.state == "CAPTURE_CART_START":
            try:
                # Try panda_hand_tcp first, fallback to panda_hand / panda_link8
                try:
                    t = self.tf_buffer.lookup_transform('panda_link0', 'panda_hand_tcp', rclpy.time.Time())
                except TransformException:
                    t = self.tf_buffer.lookup_transform('panda_link0', 'panda_hand', rclpy.time.Time())

                self.start_ee_pose = t.transform
                start_x = self.start_ee_pose.translation.x
                start_y = self.start_ee_pose.translation.y
                start_z = self.start_ee_pose.translation.z
                goal_x = start_x + self.cart_target_distance

                self.get_logger().info(
                    f"\n=========================================="
                    f"\n  Start EE Pose: X={start_x:.4f}, Y={start_y:.4f}, Z={start_z:.4f} m"
                    f"\n   Goal EE Pose: X={goal_x:.4f}, Y={start_y:.4f}, Z={start_z:.4f} m  (+{self.cart_target_distance*100:.1f} cm)"
                    f"\n=========================================="
                )
                self.state = "MOVE_CARTESIAN_X"
                self.phase_time = 0.0
            except TransformException as ex:
                self.get_logger().warn(f"Waiting for Cartesian TF: {ex}", throttle_duration_sec=1.0)
                return

        # Phase 4: Move 10 cm in +X in Cartesian space
        elif self.state == "MOVE_CARTESIAN_X":
            self.phase_time += self.timer_period
            progress = min(1.0, self.phase_time / self.cart_duration)
            s = 3.0 * (progress ** 2) - 2.0 * (progress ** 3)
            dx = s * self.cart_target_distance

            goal = CartesianImpedanceGoal()
            goal.pose.position.x = self.start_ee_pose.translation.x + dx
            goal.pose.position.y = self.start_ee_pose.translation.y
            goal.pose.position.z = self.start_ee_pose.translation.z

            goal.pose.orientation.x = self.start_ee_pose.rotation.x
            goal.pose.orientation.y = self.start_ee_pose.rotation.y
            goal.pose.orientation.z = self.start_ee_pose.rotation.z
            goal.pose.orientation.w = self.start_ee_pose.rotation.w

            goal.q_n = self.current_q
            self.cart_pub.publish(goal)

            self.get_logger().info(
                f"Phase 3: Moving EE +X: dx={dx*100:.1f} cm / {self.cart_target_distance*100:.1f} cm ({progress*100:.0f}%)",
                throttle_duration_sec=1.0
            )

            if progress >= 1.0:
                self.get_logger().info("Task Complete! Holding final Cartesian pose.")
                self.state = "TASK_COMPLETE"

        elif self.state == "TASK_COMPLETE":
            # Continue holding the final Cartesian pose
            goal = CartesianImpedanceGoal()
            goal.pose.position.x = self.start_ee_pose.translation.x + self.cart_target_distance
            goal.pose.position.y = self.start_ee_pose.translation.y
            goal.pose.position.z = self.start_ee_pose.translation.z

            goal.pose.orientation.x = self.start_ee_pose.rotation.x
            goal.pose.orientation.y = self.start_ee_pose.rotation.y
            goal.pose.orientation.z = self.start_ee_pose.rotation.z
            goal.pose.orientation.w = self.start_ee_pose.rotation.w

            goal.q_n = self.current_q
            self.cart_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
