#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
# Import the specific message types
from multi_mode_control_msgs.msg import JointGoal
from panda_motion_generator_msgs.msg import SetTimeScaling

class CircularJointMotion(Node):
    def __init__(self):
        super().__init__('circular_joint_motion')
        
        # Publisher for the joint impedance controller
        self.publisher_ = self.create_publisher(
            JointGoal, 
            '/panda/franka_joint_impedance_controller/desired_pose', 
            10
        )
        
        # Subscribe to time scaling from the Level 1 reflex
        self.time_scaling = 1.0
        self.scaling_sub_ = self.create_subscription(
            SetTimeScaling,
            '/panda/set_time_scaling',
            self.scaling_callback,
            10
        )
        
        # Control frequency (50 Hz)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.t = 0.0
        self.dt = timer_period
        
        # Base configuration
        self.q_base = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.current_q = None
        
        # Subscribe to joint states to track current position
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Circular motion parameters
        # self.amplitudes = [2.0, 0.5, 2.0, 0.5, 2.0, 2.0, 2.0]
        self.amplitudes = [0.0, 0.0, 2.0, 0.0, 2.0, 0.0, 0.0]

        self.omega = 1.7 # rad/s (increased by 2 times)
        
        self.get_logger().info("Starting circular joint motion...")

    def scaling_callback(self, msg):
        self.time_scaling = msg.scaling
        self.get_logger().info(f"Level 1 Reflex: Adjusted time scaling to {self.time_scaling}")

    def joint_state_callback(self, msg):
        # Safely extract the 7 joint positions. 
        # Fallback to the first 7 array elements if the names aren't strictly 'panda_jointX'.
        try:
            joint_map = dict(zip(msg.name, msg.position))
            self.current_q = [joint_map[f'panda_joint{i}'] for i in range(1, 8)]
        except KeyError:
            if len(msg.position) >= 7:
                self.current_q = list(msg.position[:7])

    def timer_callback(self):
        msg = JointGoal()
        
        # If we have real-time state, integrate the desired motion relative to the actual position.
        if self.current_q is not None:
            q_target = list(self.current_q)
            for i in range(7):
                phase = i * 0.5  # Add a phase offset per joint for a wavy/circular motion
                step = self.amplitudes[i] * self.omega * math.cos(self.omega * self.t + phase) * self.dt * self.time_scaling
                q_target[i] -= step  # Reversed direction to avoid immediate reflex trigger
        else:
            # Fallback before the first joint state arrives
            q_target = list(self.q_base)
            for i in range(7):
                phase = i * 0.5
                q_target[i] -= self.amplitudes[i] * (math.sin(self.omega * self.t + phase) - math.sin(phase))
        
        msg.q = q_target
        msg.qd = [0.0] * 7
        
        self.publisher_.publish(msg)
        
        # Advance time by dt multiplied by the dynamic time scaling factor
        self.t += self.dt * self.time_scaling

def main(args=None):
    rclpy.init(args=args)
    node = CircularJointMotion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping circular joint motion...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
