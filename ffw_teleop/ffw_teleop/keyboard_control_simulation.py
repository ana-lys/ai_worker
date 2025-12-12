#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Fast slider control for simulation with smart collision checking

import tkinter as tk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ffw_collision_checker.srv import CheckCollision
from urdf_parser_py.urdf import URDF
import subprocess
from ament_index_python.packages import get_package_share_directory
import os
import time


class SliderSimControl(Node):
    def run(self):
        """Main loop"""
        self.get_logger().info('Slider Simulation Control ready!')
        try:
            while rclpy.ok() and self.running:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.root.update()
        except tk.TclError:
            self.running = False

    def __init__(self):
        super().__init__('slider_sim_control')
        
        # Collision checking service client
        self.collision_client = self.create_client(CheckCollision, 'check_collision')
        self.get_logger().info('Waiting for collision checker service...')
        self.collision_client.wait_for_service(timeout_sec=5.0)
        if not self.collision_client.service_is_ready():
            self.get_logger().warn('Collision checker service not available - running without collision checking!')
        
        # Collision checking state
        self.collision_check_interval = 0.1  # 10Hz (every 100ms)
        self.last_collision_check_time = 0.0
        self.collision_prediction_horizon = 0.3  # Predict 300ms ahead
        self.last_valid_config = {}  # Track last valid (collision-free) configuration per controller
        
        # Load joint limits directly from URDF file
        self.joint_limits = self.load_joint_limits_from_urdf()

        self.controllers = {
            'arm_l': {
                'joints': [
                    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                    'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
                    'gripper_l_joint1'
                ],
                'current': [0.0] * 8,
                'target': [0.0] * 8,
                'prev_error': [0.0] * 8,
                'commanded_pos': [0.0] * 8,
                'limits': [(-3.14, 3.14)] * 8,
                'publisher': self.create_publisher(
                    JointTrajectory,
                    '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory', 10)
            },
            'arm_r': {
                'joints': [
                    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                    'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
                    'gripper_r_joint1'
                ],
                'current': [0.0] * 8,
                'target': [0.0] * 8,
                'prev_error': [0.0] * 8,
                'commanded_pos': [0.0] * 8,
                'limits': [(-3.14, 3.14)] * 8,
                'publisher': self.create_publisher(
                    JointTrajectory,
                    '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory', 10)
            },
            'head': {
                'joints': ['head_joint1', 'head_joint2'],
                'current': [0.0] * 2,
                'target': [0.0] * 2,
                'prev_error': [0.0] * 2,
                'commanded_pos': [0.0] * 2,
                'limits': [(-1.0, 1.0)] * 2,
                'publisher': self.create_publisher(
                    JointTrajectory, '/leader/joystick_controller_left/joint_trajectory', 10)
            },
            'lift': {
                'joints': ['lift_joint'],
                'current': [0.0],
                'target': [0.0],
                'prev_error': [0.0],
                'commanded_pos': [0.0],
                'limits': [(-1.0, 0.0)],
                'publisher': self.create_publisher(
                    JointTrajectory, '/leader/joystick_controller_right/joint_trajectory', 10)
            }
        }
        
        # Populate joint limits from URDF
        for ctrl_key, ctrl in self.controllers.items():
            ctrl['limits'] = [
                self.joint_limits.get(joint, (-3.14, 3.14))
                for joint in ctrl['joints']
            ]
            self.get_logger().info(f'Controller {ctrl_key} limits: {list(zip(ctrl["joints"], ctrl["limits"]))}')
            # Initialize last valid config to zero/home position
            self.last_valid_config[ctrl_key] = [0.0] * len(ctrl['joints'])
        
        # PD controller gains
        self.kp = 5.0  # Proportional gain
        self.kd = 0.5  # Derivative gain
        self.dt = 0.02  # 50Hz = 0.02s
        self.v_max = 2.0  # rad/s clamp for commanded velocity

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.running = True
        self.sliders = {}
        self.value_labels = {}
        self.dragging = set()
        self.gui_updating = False  # suppress on_slider_drag during programmatic updates

        self.root = tk.Tk()
        self.root.title('Slider Simulation Control')
        self.build_gui()
        
        # Control loop - 20ms = 50Hz for smooth streaming
        self.root.after(int(self.dt * 1000), self.control_loop)

    def joint_state_callback(self, msg):
        """Update current positions from feedback"""
        for ctrl_key, ctrl in self.controllers.items():
            for i, joint in enumerate(ctrl['joints']):
                if joint in msg.name:
                    idx = msg.name.index(joint)
                    ctrl['current'][i] = msg.position[idx]

    def on_slider_drag(self, ctrl_key, joint_idx, value):
        """Called while dragging slider"""
        if self.gui_updating:
            return
        self.dragging.add((ctrl_key, joint_idx))
        ctrl = self.controllers[ctrl_key]
        ctrl['target'][joint_idx] = float(value)
        if (ctrl_key, joint_idx) in self.value_labels:
            self.value_labels[(ctrl_key, joint_idx)].config(text=f'{float(value):.2f}')

    def on_slider_release(self, event, ctrl_key, joint_idx):
        """Called when slider is released"""
        self.dragging.discard((ctrl_key, joint_idx))

    def should_check_collision(self, ctrl_key, predicted_positions):
        """Determine if collision check is needed based on time"""
        current_time = time.time()
        
        # Time-based check (10Hz)
        time_since_last_check = current_time - self.last_collision_check_time
        if time_since_last_check < self.collision_check_interval:
            return False
        
        return True

    def predict_future_position(self, ctrl_key, horizon_time):
        """Predict joint positions after horizon_time seconds"""
        ctrl = self.controllers[ctrl_key]
        predicted = []
        
        for i in range(len(ctrl['joints'])):
            # Current commanded position + velocity * horizon
            if 'prev_velocity' in ctrl:
                v_current = ctrl['prev_velocity'][i]
            else:
                v_current = 0.0
            
            # Predict: p_future = p_current + v * t
            p_future = ctrl['commanded_pos'][i] + v_current * horizon_time
            
            # Clamp to joint limits
            min_limit, max_limit = ctrl['limits'][i]
            p_future = max(min_limit, min(max_limit, p_future))
            
            predicted.append(p_future)
        
        return predicted

    def check_collision(self, joint_names, joint_positions):
        """Check if given joint configuration causes collision"""
        request = CheckCollision.Request()
        request.joint_names = joint_names
        request.joint_positions = joint_positions
        
        try:
            future = self.collision_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.05)  # Longer timeout for 10Hz
            if future.done():
                response = future.result()
                if response.in_collision:
                    self.get_logger().info(f'Collision! min_distance={response.min_distance:.4f}')
                return response.in_collision
            else:
                self.get_logger().warn('Collision check timeout')
                return False  # Allow movement if check times out
        except Exception as e:
            self.get_logger().error(f'Collision check failed: {e}')
            return False  # Allow movement on error

    def control_loop(self):
        """Control loop with PD controller - publish p/v/a at 50Hz"""
        threshold = 0.01  # Small deadband
        
        for ctrl_key, ctrl in self.controllers.items():
            needs_update = any(
                abs(ctrl['target'][i] - ctrl['current'][i]) > threshold
                for i in range(len(ctrl['joints']))
            )
            
            if needs_update:
                traj = JointTrajectory()
                traj.joint_names = ctrl['joints']
                
                # Initialize histories on first update
                if 'prev_velocity' not in ctrl:
                    ctrl['prev_velocity'] = [0.0] * len(ctrl['joints'])
                    for i in range(len(ctrl['joints'])):
                        ctrl['commanded_pos'][i] = ctrl['current'][i]

                positions = []
                velocities = []
                accelerations = []

                for i in range(len(ctrl['joints'])):
                    error = ctrl['target'][i] - ctrl['current'][i]
                    d_error = (error - ctrl['prev_error'][i]) / self.dt
                    
                    # PD control to compute desired velocity
                    v_cmd = self.kp * error + self.kd * d_error
                    # Clamp
                    v_cmd = max(-self.v_max, min(self.v_max, v_cmd))
                    
                    # Derive acceleration from velocity change
                    a_cmd = (v_cmd - ctrl['prev_velocity'][i]) / self.dt

                    # Increment from LAST COMMANDED position, not current feedback
                    p_cmd = ctrl['commanded_pos'][i] + v_cmd * self.dt

                    if abs(error) > 0.1:
                        print(" %i %f %f %f" % (i, p_cmd , v_cmd, a_cmd))
                    
                    positions.append(p_cmd)
                    velocities.append(v_cmd)
                    accelerations.append(a_cmd)

                    # Update histories
                    ctrl['prev_error'][i] = error
                    ctrl['prev_velocity'][i] = v_cmd
                    ctrl['commanded_pos'][i] = p_cmd

                # Smart collision checking: predict future position and check at 10Hz
                if self.collision_client.service_is_ready():
                    # Predict where we'll be after the horizon
                    predicted_pos = self.predict_future_position(ctrl_key, self.collision_prediction_horizon)
                    
                    if self.should_check_collision(ctrl_key, predicted_pos):
                        if self.check_collision(ctrl['joints'], predicted_pos):
                            self.get_logger().warn(f'Collision predicted for {ctrl_key}! Rolling back to last valid config.')
                            
                            # Rollback: restore both target and GUI sliders to last valid config
                            last_valid = self.last_valid_config[ctrl_key]
                            for i in range(len(ctrl['joints'])):
                                ctrl['target'][i] = last_valid[i]
                                # Update GUI slider to reflect rollback
                                if (ctrl_key, i) in self.sliders:
                                    self.gui_updating = True
                                    self.sliders[(ctrl_key, i)].set(last_valid[i])
                                    self.gui_updating = False
                                if (ctrl_key, i) in self.value_labels:
                                    self.value_labels[(ctrl_key, i)].config(text=f'{last_valid[i]:.2f}')
                            continue
                        
                        # Update last check time and save as valid config
                        self.last_collision_check_time = time.time()
                        self.last_valid_config[ctrl_key] = predicted_pos.copy()
                
                point = JointTrajectoryPoint()
                point.positions = positions
                point.velocities = velocities
                point.accelerations = accelerations
                point.time_from_start.sec = 0
                point.time_from_start.nanosec = 0

                traj.points.append(point)
                ctrl['publisher'].publish(traj)

        # Schedule next loop
        self.root.after(int(self.dt * 1000), self.control_loop)

    def build_gui(self):
        title = tk.Label(
            self.root,
            text='🎮 SLIDER SIMULATION CONTROL 🎮\n0.02s horizon | 50Hz control | 10Hz collision check',
            font=('Arial', 11, 'bold'),
            fg='white',
            bg='green',
            pady=10
        )
        title.grid(row=0, column=0, columnspan=4, sticky='ew')
        
        row = 1
        for ctrl_key, ctrl in self.controllers.items():
            header = tk.Label(
                self.root,
                text=ctrl_key.upper(),
                font=('Arial', 12, 'bold'),
                bg='lightgray',
                pady=5
            )
            header.grid(row=row, column=0, columnspan=4, sticky='ew')
            row += 1
            
            for i, joint in enumerate(ctrl['joints']):
                min_limit, max_limit = ctrl['limits'][i]

                name_label = tk.Label(self.root, text=joint, width=20, anchor='w')
                name_label.grid(row=row, column=0, padx=5, pady=2, sticky='w')

                min_label = tk.Label(self.root, text=f'{min_limit:.2f}', width=6, anchor='e', fg='blue')
                min_label.grid(row=row, column=1, padx=(5,0), pady=2, sticky='e')

                slider = tk.Scale(
                    self.root,
                    from_=min_limit,
                    to=max_limit,
                    resolution=0.01,
                    orient=tk.HORIZONTAL,
                    length=250,
                    showvalue=False,
                    command=lambda val, c=ctrl_key, j=i: self.on_slider_drag(c, j, val)
                )
                slider.set(0.0)
                slider.grid(row=row, column=2, padx=0, pady=2)
                slider.bind('<ButtonRelease-1>', 
                           lambda e, c=ctrl_key, j=i: self.on_slider_release(e, c, j))
                self.sliders[(ctrl_key, i)] = slider

                max_label = tk.Label(self.root, text=f'{max_limit:.2f}', width=6, anchor='w', fg='red')
                max_label.grid(row=row, column=3, padx=(0,5), pady=2, sticky='w')

                value_label = tk.Label(self.root, text='0.00', width=8, font=('Courier', 10))
                value_label.grid(row=row, column=4, padx=5, pady=2)
                self.value_labels[(ctrl_key, i)] = value_label

                row += 1
        
        reset_btn = tk.Button(
            self.root,
            text='RESET ALL TO ZERO',
            command=self.reset_all,
            bg='orange',
            font=('Arial', 10, 'bold'),
            pady=5
        )
        reset_btn.grid(row=row, column=0, columnspan=4, pady=10, sticky='ew')

    def reset_all(self):
        """Reset all joints to zero"""
        for ctrl_key, ctrl in self.controllers.items():
            for i in range(len(ctrl['joints'])):
                min_limit, max_limit = ctrl['limits'][i]
                zero = max(min(0.0, max_limit), min_limit)
                ctrl['target'][i] = zero
                if (ctrl_key, i) in self.sliders:
                    self.sliders[(ctrl_key, i)].set(zero)
                if (ctrl_key, i) in self.value_labels:
                    self.value_labels[(ctrl_key, i)].config(text=f'{zero:.2f}')
        self.get_logger().info('Reset all joints')

    def load_joint_limits_from_urdf(self):
        """Load joint limits directly from URDF file"""
        limits = {}
        package_share_directory = get_package_share_directory('ffw_description')
    
        # Construct the full path to your URDF file
        urdf_file = os.path.join(package_share_directory, 'urdf', 'ffw_sg2_rev1_follower', 'ffw_sg2_follower.urdf')
        try:
            if not os.path.exists(urdf_file):
                self.get_logger().warn(f'URDF file not found: {urdf_file}')
                return limits
            self.get_logger().info(f'Loading URDF from: {urdf_file}')
            with open(urdf_file, 'r') as f:
                urdf_string = f.read()
            robot = URDF.from_xml_string(urdf_string)
            for joint in robot.joints:
                if joint.limit is not None:
                    limits[joint.name] = (joint.limit.lower, joint.limit.upper)
                    self.get_logger().info(f'  {joint.name}: ({joint.limit.lower:.4f}, {joint.limit.upper:.4f})')
            self.get_logger().info(f'Loaded limits for {len(limits)} joints from URDF')
        except Exception as e:
            self.get_logger().error(f'Failed to load joint limits: {e}')
        return limits


def main():
    rclpy.init()
    node = SliderSimControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()