#!/usr/bin/env python3
"""
Normal (No-Reflex) Controller Trajectory Task Script.
Queries controller_manager to inspect, load, configure, and switch controllers:
- Loads & configures standalone joint_impedance_controller & cartesian_impedance_controller dynamically if not loaded.
- Deactivates reflex_mc_controller (or other active motion controllers) if active.
- Activates standalone joint_impedance_controller to move to P1 & P2:
  P1: [1.44267, 0.5971, 0.262509, -2.04938, -0.317762, 2.60155, -0.472003]
  P2: [1.49287, 0.933638, 0.150206, -1.89913, -0.349659, 2.81977, -0.481786]
- Activates standalone cartesian_impedance_controller to move 15 cm along +X.
- Publishes exclusively to standalone controller topics:
  - Joint: /joint_impedance/panda_joints_desired (sensor_msgs/JointState)
  - Cartesian: /cartesian_impedance/pose_desired (std_msgs/Float64MultiArray)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, ListControllers, LoadController, ConfigureController
from tf2_ros import Buffer, TransformListener, TransformException


class MoveTaskNormal(Node):
    def __init__(self):
        super().__init__('move_task_normal')

        # Target joint positions
        self.q1 = [1.44267, 0.5971, 0.262509, -2.04938, -0.317762, 2.60155, -0.472003]
        self.q2 = [1.34507,0.871697,0.165614,-1.99556,-0.388187,2.83158,-0.584412]

        # Controller Manager Service Clients
        self.list_ctrl_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )
        self.load_ctrl_client = self.create_client(
            LoadController,
            '/controller_manager/load_controller'
        )
        self.config_ctrl_client = self.create_client(
            ConfigureController,
            '/controller_manager/configure_controller'
        )
        self.switch_ctrl_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

        # Standalone Controller Publishers ONLY
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_impedance/panda_joints_desired',
            10
        )
        self.cart_pub = self.create_publisher(
            Float64MultiArray,
            '/cartesian_impedance/pose_desired',
            10
        )

        # TF Listener for Cartesian EE pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Joint State Subscriber
        self.current_q = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Execution State Machine
        self.state = "INIT"
        self.q_start_motion = None
        self.start_ee_pose = None

        self.phase_duration = 2.0         # 2 seconds per joint transition
        self.cart_duration = 1.0          # 2 seconds for Cartesian +X motion
        self.cart_target_distance = 0.2  # 15 cm along +X
        self.timer_period = 0.01         # 200 Hz (5 ms period)
        self.phase_time = 0.0

        self.get_logger().info("Initializing Normal Task Script (Dynamic Controller Load & Switch)...")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def joint_callback(self, msg):
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.current_q = [joint_map[f'panda_joint{i}'] for i in range(1, 8)]
        except (KeyError, ValueError):
            if len(msg.position) >= 7:
                self.current_q = list(msg.position[:7])

    def publish_joint_goal(self, q_goal):
        js_msg = JointState()
        js_msg.name = [f'panda_joint{i}' for i in range(1, 8)]
        js_msg.position = q_goal
        self.joint_pub.publish(js_msg)

    def quat_to_matrix(self, qx, qy, qz, qw):
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm > 0.0:
            x, y, z, w = qx/norm, qy/norm, qz/norm, qw/norm
        else:
            x, y, z, w = 0.0, 0.0, 0.0, 1.0

        r11 = 1.0 - 2.0 * (y*y + z*z)
        r12 = 2.0 * (x*y - w*z)
        r13 = 2.0 * (x*z + w*y)

        r21 = 2.0 * (x*y + w*z)
        r22 = 1.0 - 2.0 * (x*x + z*z)
        r23 = 2.0 * (y*z - w*x)

        r31 = 2.0 * (x*z - w*y)
        r32 = 2.0 * (y*z + w*x)
        r33 = 1.0 - 2.0 * (x*x + y*y)

        return [r11, r12, r13, r21, r22, r23, r31, r32, r33]

    def publish_cartesian_goal(self, px, py, pz, qx, qy, qz, qw):
        rot_flat = self.quat_to_matrix(qx, qy, qz, qw)
        data = [px, py, pz] + rot_flat

        arr_msg = Float64MultiArray()
        arr_msg.data = data
        self.cart_pub.publish(arr_msg)

    def switch_controller(self, activate_name, next_state):
        self.get_logger().info(f"Inspecting controllers before activating '{activate_name}'...")
        if not self.list_ctrl_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/controller_manager/list_controllers service unavailable!")
            return False

        list_req = ListControllers.Request()
        list_future = self.list_ctrl_client.call_async(list_req)

        def on_list_done(fut):
            try:
                controllers = fut.result().controller
                loaded_ctrl_names = [c.name for c in controllers]
                active_ctrls = [c.name for c in controllers if c.state == "active"]
                self.get_logger().info(f"Current loaded controllers: {loaded_ctrl_names}, Active: {active_ctrls}")

                # Step 1: If controller is not loaded, load it first
                if activate_name not in loaded_ctrl_names:
                    self.get_logger().info(f"Controller '{activate_name}' is not loaded. Calling load_controller...")
                    if self.load_ctrl_client.wait_for_service(timeout_sec=2.0):
                        load_req = LoadController.Request()
                        load_req.name = activate_name
                        load_fut = self.load_ctrl_client.call_async(load_req)

                        def on_load_done(lfut):
                            try:
                                res = lfut.result()
                                if res and res.ok:
                                    self.get_logger().info(f"Successfully loaded '{activate_name}'. Configuring...")
                                    self.configure_and_activate(activate_name, active_ctrls, next_state)
                                else:
                                    self.get_logger().error(f"Failed to load controller '{activate_name}'.")
                            except Exception as e:
                                self.get_logger().error(f"LoadController exception: {e}")

                        load_fut.add_done_callback(on_load_done)
                        return
                    else:
                        self.get_logger().error("/controller_manager/load_controller service unavailable!")
                        return

                # Step 2: If controller is loaded, configure and activate
                self.configure_and_activate(activate_name, active_ctrls, next_state)

            except Exception as e:
                self.get_logger().error(f"ListControllers call failed: {e}")

        list_future.add_done_callback(on_list_done)
        return True

    def configure_and_activate(self, activate_name, active_ctrls, next_state):
        if self.config_ctrl_client.wait_for_service(timeout_sec=1.0):
            cfg_req = ConfigureController.Request()
            cfg_req.name = activate_name
            cfg_fut = self.config_ctrl_client.call_async(cfg_req)

            def on_cfg_done(cfut):
                self.do_switch(activate_name, active_ctrls, next_state)

            cfg_fut.add_done_callback(on_cfg_done)
        else:
            self.do_switch(activate_name, active_ctrls, next_state)

    def do_switch(self, activate_name, active_ctrls, next_state):
        deactivate_list = [c for c in active_ctrls if c not in [
            'joint_state_broadcaster',
            'franka_robot_state_broadcaster',
            'franka_robot_model_broadcaster',
            activate_name
        ]]

        if activate_name in active_ctrls and len(deactivate_list) == 0:
            self.get_logger().info(f"Controller '{activate_name}' is already active.")
            self.state = next_state
            self.phase_time = 0.0
            self.q_start_motion = list(self.current_q) if self.current_q else list(self.q1)
            return

        self.get_logger().info(f"Activating '{activate_name}' and deactivating {deactivate_list}...")

        if not self.switch_ctrl_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/controller_manager/switch_controller service unavailable!")
            return

        switch_req = SwitchController.Request()
        switch_req.activate_controllers = [activate_name]
        switch_req.deactivate_controllers = deactivate_list
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        switch_req.activate_asap = True

        switch_future = self.switch_ctrl_client.call_async(switch_req)

        def on_switch_done(sfut):
            try:
                res = sfut.result()
                if res.ok:
                    self.get_logger().info(f"Successfully activated '{activate_name}'.")
                    self.state = next_state
                    self.phase_time = 0.0
                    self.q_start_motion = list(self.current_q) if self.current_q else list(self.q1)
                else:
                    self.get_logger().error(f"Failed to activate '{activate_name}'.")
            except Exception as e:
                self.get_logger().error(f"SwitchController call failed: {e}")

        switch_future.add_done_callback(on_switch_done)

    def timer_callback(self):
        if self.current_q is None:
            self.get_logger().info("Waiting for initial /joint_states...", throttle_duration_sec=1.0)
            return

        # Phase 0: Start sequence -> Load/Configure/Switch to joint_impedance_controller
        if self.state == "INIT":
            self.state = "SWITCHING_TO_JOINT"
            self.switch_controller(
                activate_name="joint_impedance_controller",
                next_state="MOVE_TO_P1"
            )
            return

        elif self.state == "SWITCHING_TO_JOINT":
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

            self.publish_joint_goal(q_goal)

            self.get_logger().info(
                f"[Normal Task] Phase 1: Moving to Joint P1 ({progress*100:.0f}%)",
                throttle_duration_sec=1.0
            )

            if progress >= 1.0:
                self.get_logger().info("Reached Joint Position 1!")
                self.state = "PREPARE_MOVE_TO_P2"
                self.phase_time = 0.0

        elif self.state == "PREPARE_MOVE_TO_P2":
            self.phase_time += self.timer_period
            if self.phase_time >= 0.5:
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

            self.publish_joint_goal(q_goal)

            self.get_logger().info(
                f"[Normal Task] Phase 2: Moving to Joint P2 ({progress*100:.0f}%)",
                throttle_duration_sec=1.0
            )

            if progress >= 1.0:
                self.get_logger().info("Reached Joint Position 2!")
                self.state = "PREPARE_SWITCH_TO_CART"
                self.phase_time = 0.0

        elif self.state == "PREPARE_SWITCH_TO_CART":
            self.phase_time += self.timer_period
            if self.phase_time >= 0.5:
                self.state = "SWITCHING_TO_CART"
                self.switch_controller(
                    activate_name="cartesian_impedance_controller",
                    next_state="CAPTURE_CART_START"
                )

        elif self.state == "SWITCHING_TO_CART":
            return

        # Phase 3: Capture starting Cartesian EE pose from TF
        elif self.state == "CAPTURE_CART_START":
            try:
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
                    f"\n  [Normal Task] Start EE Pose: X={start_x:.4f}, Y={start_y:.4f}, Z={start_z:.4f} m"
                    f"\n  [Normal Task]  Goal EE Pose: X={goal_x:.4f}, Y={start_y:.4f}, Z={start_z:.4f} m  (+{self.cart_target_distance*100:.1f} cm)"
                    f"\n=========================================="
                )
                self.state = "MOVE_CARTESIAN_X"
                self.phase_time = 0.0
            except TransformException as ex:
                self.get_logger().warn(f"Waiting for Cartesian TF: {ex}", throttle_duration_sec=1.0)
                return

        # Phase 4: Move 15 cm in +X in Cartesian space
        elif self.state == "MOVE_CARTESIAN_X":
            self.phase_time += self.timer_period
            progress = min(1.0, self.phase_time / self.cart_duration)
            s = 3.0 * (progress ** 2) - 2.0 * (progress ** 3)
            dx = s * self.cart_target_distance

            px = self.start_ee_pose.translation.x + dx
            py = self.start_ee_pose.translation.y
            pz = self.start_ee_pose.translation.z

            qx = self.start_ee_pose.rotation.x
            qy = self.start_ee_pose.rotation.y
            qz = self.start_ee_pose.rotation.z
            qw = self.start_ee_pose.rotation.w

            self.publish_cartesian_goal(px, py, pz, qx, qy, qz, qw)

            self.get_logger().info(
                f"[Normal Task] Phase 3: Moving EE +X: dx={dx*100:.1f} cm / {self.cart_target_distance*100:.1f} cm ({progress*100:.0f}%)",
                throttle_duration_sec=1.0
            )

            if progress >= 1.0:
                self.get_logger().info("Normal Task Complete! Holding final Cartesian pose.")
                self.state = "TASK_COMPLETE"

        elif self.state == "TASK_COMPLETE":
            px = self.start_ee_pose.translation.x + self.cart_target_distance
            py = self.start_ee_pose.translation.y
            pz = self.start_ee_pose.translation.z

            qx = self.start_ee_pose.rotation.x
            qy = self.start_ee_pose.rotation.y
            qz = self.start_ee_pose.rotation.z
            qw = self.start_ee_pose.rotation.w

            self.publish_cartesian_goal(px, py, pz, qx, qy, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTaskNormal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
