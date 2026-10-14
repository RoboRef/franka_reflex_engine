#!/usr/bin/env python3
"""
Figure-8 example for the franka_reflex_engine (real robot).

Sequence
--------
1. Switch to ``franka_move_to_start_controller``. It drives the arm to
   ``<arm_id>.recovery_goal_position`` (same goal the reflex engine uses for
   recovery, configured in the YAML). The node waits until the arm has reached
   that goal and settled.
2. Switch to ``franka_cartesian_impedance_controller``, capture the current
   end-effector pose (O_T_EE from the Franka robot state broadcaster) and use
   it as the centre of the figure-8.
3. Publish a figure-8 (lemniscate of Gerono) in the XY plane of the robot base:

       y(t) = y0 + A_y * sin(theta(t))          (long axis, lateral)
       x(t) = x0 + A_x * sin(2 * theta(t))      (short axis, forward/back)

   with z and orientation held constant. theta(t) is ramped up smoothly so
   the trajectory starts with zero velocity.

Reflex handling
---------------
The node listens on ``/<arm_id>/safety_engine_state``. As soon as the reflex
engine leaves NORMAL (REFLEX / WAIT_FOR_RECOVERY / RECOVERY) the node stops
publishing goals. When the engine reports NORMAL again (after haptic or
service recovery), the node waits ``resume_delay`` seconds and restarts the
whole sequence from step 1.

Trigger recovery manually with:
    ros2 service call /panda/recover std_srvs/srv/Empty
"""

import math

import rclpy
from rclpy.node import Node

from franka_msgs.msg import FrankaState
from multi_mode_control_msgs.msg import CartesianImpedanceGoal, Controller
from multi_mode_control_msgs.srv import SetCartesianImpedance, SetControllers
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String


MOVE_TO_START_CTRL = 'franka_move_to_start_controller'
CART_CTRL = 'franka_cartesian_impedance_controller'


def rotation_to_quaternion(r):
    """Convert a 3x3 rotation matrix (list of rows) to (x, y, z, w)."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2][1] - r[1][2]) * s
        y = (r[0][2] - r[2][0]) * s
        z = (r[1][0] - r[0][1]) * s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2])
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2])
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1])
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


class FigureEightPublisher(Node):
    def __init__(self):
        super().__init__('figure_eight_publisher')

        # ── Parameters ────────────────────────────────────────────────────
        self.arm_id = self.declare_parameter('arm_id', 'panda').value
        self.controller_node = self.declare_parameter(
            'controller_node', 'reflex_multimode_controller').value
        self.amplitude_x = float(self.declare_parameter('amplitude_x', 0.08).value)  # m (short axis)
        self.amplitude_y = float(self.declare_parameter('amplitude_y', 0.15).value)  # m (long axis)
        self.period = float(self.declare_parameter('period', 5.0).value)            # s / loop
        self.ramp_time = float(self.declare_parameter('ramp_time', 1.0).value)       # s
        self.num_loops = int(self.declare_parameter('num_loops', 0).value)           # 0 = forever
        self.resume_delay = float(self.declare_parameter('resume_delay', 2.0).value) # s
        self.rate = float(self.declare_parameter('publish_rate', 100.0).value)       # Hz
        # Move-to-start convergence criteria
        self.goal_tolerance = float(self.declare_parameter('goal_tolerance', 0.10).value)     # rad, sum |q - q_goal|
        self.settle_velocity = float(self.declare_parameter('settle_velocity', 0.2).value)   # rad/s, sum |dq|
        self.settle_time = float(self.declare_parameter('settle_time', 0.5).value)            # s
        self.move_to_start_timeout = float(
            self.declare_parameter('move_to_start_timeout', 10.0).value)                       # s

        self.dt = 1.0 / self.rate
        self.omega = 2.0 * math.pi / self.period

        # ── ROS interfaces ────────────────────────────────────────────────
        self.set_ctrl_client = self.create_client(
            SetControllers, f'/{self.controller_node}/set_controllers')
        self.set_cart_params_client = self.create_client(
            SetCartesianImpedance, f'/{self.arm_id}/{CART_CTRL}/parameters')
        self.get_param_client = self.create_client(
            GetParameters, f'/{self.controller_node}/get_parameters')
        self.cart_pub = self.create_publisher(
            CartesianImpedanceGoal, f'/{self.arm_id}/{CART_CTRL}/desired_pose', 10)

        self.create_subscription(
            FrankaState, '/franka_robot_state_broadcaster/robot_state', self._state_cb, 10)
        self.create_subscription(
            String, f'/{self.arm_id}/safety_engine_state', self._safety_cb, 10)

        # ── Internal state ────────────────────────────────────────────────
        self.q = None
        self.dq = None
        self.o_t_ee = None
        self.q_goal = [0.0, -0.785, 0.0, -2.35619, 0.0, 1.571, 0.785]
        self._goal_fetched = False
        self.engine_state = 'NORMAL'
        self.state = 'INIT'
        self.t = 0.0
        self.traj_t = 0.0
        self.max_lead = 0.08      # m (8 cm max lead: strictly < 10 cm controller threshold)
        self.settled_for = 0.0
        self.center = None        # (x, y, z)
        self.orientation = None   # (x, y, z, w)
        self.q_n = None           # nullspace posture for the Cartesian controller

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)

        self.get_logger().info(
            f'Figure-8: A_x={self.amplitude_x:.3f} m, A_y={self.amplitude_y:.3f} m, '
            f'period={self.period:.1f} s, loops={"inf" if self.num_loops == 0 else self.num_loops}, '
            f'use_sim_time={self.use_sim_time}')
        self.timer = self.create_timer(self.dt, self._step)

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _state_cb(self, msg: FrankaState):
        self.q = list(msg.q)
        self.dq = list(msg.dq)
        self.o_t_ee = list(msg.o_t_ee)

    def _safety_cb(self, msg: String):
        prev, self.engine_state = self.engine_state, msg.data
        if prev == self.engine_state:
            return
        self.get_logger().info(f'Reflex engine state: {prev} -> {self.engine_state}')
        if self.engine_state != 'NORMAL':
            if self.state not in ('PAUSED', 'DONE'):
                self.get_logger().warn('Reflex active - pausing figure-8 task.')
            self._set_state('PAUSED')
        elif self.state == 'PAUSED':
            self.get_logger().info(
                f'Recovery complete - restarting task in {self.resume_delay:.1f} s.')
            self._set_state('RESUME_WAIT')

    # ── Helpers ───────────────────────────────────────────────────────────
    def _set_state(self, new_state):
        self.state = new_state
        self.t = 0.0
        self.traj_t = 0.0
        self.settled_for = 0.0

    def _configure_cartesian_stiffness(self, trans_stiffness=1000.0, rot_stiffness=30.0):
        """Increase Cartesian stiffness so the arm tracks faster with lower lag."""
        if not self.set_cart_params_client.service_is_ready():
            return
        req = SetCartesianImpedance.Request()
        stiffness = [0.0] * 36
        for i in range(3):
            stiffness[i * 6 + i] = trans_stiffness
        for i in range(3, 6):
            stiffness[i * 6 + i] = rot_stiffness
        req.stiffness = stiffness
        req.damping_ratio = [0.8] * 6
        req.nullspace_stiffness = 10.0
        self.set_cart_params_client.call_async(req)
        self.get_logger().info(f'Configured Cartesian translational stiffness: {trans_stiffness:.0f} N/m.')

    def _fetch_goal(self):
        """Read <arm_id>.recovery_goal_position from the controller node (best effort)."""
        if not self.get_param_client.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = [f'{self.arm_id}.recovery_goal_position']
        future = self.get_param_client.call_async(req)

        def done(fut):
            try:
                values = fut.result().values
                if values and values[0].type == ParameterType.PARAMETER_DOUBLE_ARRAY \
                        and len(values[0].double_array_value) == 7:
                    self.q_goal = list(values[0].double_array_value)
                    self._goal_fetched = True
                    self.get_logger().info(
                        'Move-to-start goal: [' +
                        ', '.join(f'{v:.3f}' for v in self.q_goal) + ']')
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'Could not read recovery_goal_position: {e}')
        future.add_done_callback(done)

    def _switch_controller(self, name, next_state):
        if not self.set_ctrl_client.service_is_ready():
            self.get_logger().warn('Waiting for set_controllers service...',
                                   throttle_duration_sec=2.0)
            return
        req = SetControllers.Request()
        ctrl = Controller()
        ctrl.name = name
        ctrl.resources = [self.arm_id]
        req.controllers = [ctrl]
        self.get_logger().info(f"Switching to '{name}'...")
        self._set_state('SWITCHING')
        future = self.set_ctrl_client.call_async(req)

        def done(fut):
            if self.state != 'SWITCHING':
                return  # A reflex fired in the meantime
            try:
                fut.result()
                self._set_state(next_state)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'Controller switch failed: {e}')
                self._set_state('PAUSED')
        future.add_done_callback(done)

    def _reached_start(self):
        """True once the arm is at the goal (if known) and has been still for settle_time."""
        speed = sum(abs(v) for v in self.dq)
        at_goal = True
        if self.q_goal is not None:
            at_goal = sum(abs(a - b) for a, b in zip(self.q, self.q_goal)) < self.goal_tolerance
        if at_goal and speed < self.settle_velocity:
            self.settled_for += self.dt
        else:
            self.settled_for = 0.0
        # Without a known goal, require at least 1 s of motion time before trusting "settled".
        min_time = 0.0 if self.q_goal is not None else 1.0
        return self.t > min_time and self.settled_for >= self.settle_time

    def _theta(self, t):
        """Phase with a quadratic ramp-up so the motion starts at rest."""
        if t < self.ramp_time:
            return self.omega * t * t / (2.0 * self.ramp_time)
        return self.omega * (t - 0.5 * self.ramp_time)

    def _publish_cartesian(self, x, y, z):
        goal = CartesianImpedanceGoal()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = self.orientation
        goal.q_n = list(self.q_n)
        self.cart_pub.publish(goal)

    # ── State machine ─────────────────────────────────────────────────────
    def _step(self):
        if self.o_t_ee is None:
            self.get_logger().info('Waiting for Franka robot state...',
                                   throttle_duration_sec=5.0)
            return

        self.t += self.dt

        if self.state in ('PAUSED', 'SWITCHING', 'DONE'):
            return

        if self.state == 'INIT':
            if self.engine_state != 'NORMAL':
                return
            if not self._goal_fetched:
                self._fetch_goal()
            self._switch_controller(MOVE_TO_START_CTRL, 'MOVE_TO_START')

        elif self.state == 'RESUME_WAIT':
            if self.t >= self.resume_delay:
                self._set_state('INIT')

        elif self.state == 'MOVE_TO_START':
            dist = sum(abs(a - b) for a, b in zip(self.q, self.q_goal)) if self.q_goal is not None else 0.0
            self.get_logger().info(
                f'Moving to start configuration... (dist={dist:.3f} rad, tol={self.goal_tolerance:.3f})',
                throttle_duration_sec=3.0)
            if self._reached_start():
                self.get_logger().info('Start configuration reached.')
                self._switch_controller(CART_CTRL, 'CAPTURE_CENTER')
            elif self.t > self.move_to_start_timeout:
                self.get_logger().error(
                    f'Move-to-start did not converge within {self.move_to_start_timeout:.0f} s '
                    f'(current: {[round(x, 3) for x in self.q]}, goal: {[round(x, 3) for x in self.q_goal]}, dist: {dist:.3f}). Task stopped.')
                self._set_state('DONE')

        elif self.state == 'CAPTURE_CENTER':
            if self.t < 0.3:  # let the Cartesian controller settle and state refresh
                return
            self._configure_cartesian_stiffness(1000.0, 30.0)
            m = self.o_t_ee  # column-major 4x4
            rot = [[m[0], m[4], m[8]],
                   [m[1], m[5], m[9]],
                   [m[2], m[6], m[10]]]
            self.center = (m[12], m[13], m[14])
            self.orientation = rotation_to_quaternion(rot)
            self.q_n = list(self.q)
            self.get_logger().info(
                f'Figure-8 centre: x={self.center[0]:.3f}, y={self.center[1]:.3f}, '
                f'z={self.center[2]:.3f} m. Starting trajectory.')
            self._set_state('FIGURE_EIGHT')

        elif self.state == 'FIGURE_EIGHT':
            # 1. Ideal trajectory point at phase traj_t
            theta = self._theta(self.traj_t)
            x0, y0, z0 = self.center
            ideal_x = x0 + self.amplitude_x * math.sin(2.0 * theta)
            ideal_y = y0 + self.amplitude_y * math.sin(theta)
            ideal_z = z0

            # 2. Actual robot end-effector position
            m = self.o_t_ee
            curr_x, curr_y, curr_z = m[12], m[13], m[14]

            # 3. Tracking lag / lead distance
            dx = ideal_x - curr_x
            dy = ideal_y - curr_y
            dz = ideal_z - curr_z
            lead = math.sqrt(dx * dx + dy * dy + dz * dz)

            # 4. Carrot clamping: NEVER command further than max_lead (0.08 m < 0.10 m)
            # This strictly prevents the controller from discarding target poses!
            if lead > self.max_lead:
                scale = self.max_lead / lead
                cmd_x = curr_x + dx * scale
                cmd_y = curr_y + dy * scale
                cmd_z = curr_z + dz * scale
            else:
                cmd_x = ideal_x
                cmd_y = ideal_y
                cmd_z = ideal_z

            self._publish_cartesian(cmd_x, cmd_y, cmd_z)

            # 5. Progress-gating: only advance trajectory clock if robot is keeping up
            # If lead >= max_lead, pause the clock and let the robot catch up.
            if lead < 0.05:
                self.traj_t += self.dt
            elif lead < self.max_lead:
                speed_factor = (self.max_lead - lead) / (self.max_lead - 0.05)
                self.traj_t += self.dt * speed_factor
            else:
                pass  # pause trajectory clock until robot catches up

            loops = theta / (2.0 * math.pi)
            self.get_logger().info(
                f'Figure-8 loop {loops:.2f}',
                throttle_duration_sec=3.0)
            if self.num_loops > 0 and loops >= self.num_loops:
                self.get_logger().info('Finished all loops - holding centre pose.')
                self._publish_cartesian(x0, y0, z0)
                self._set_state('DONE')


def main(args=None):
    rclpy.init(args=args)
    node = FigureEightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
