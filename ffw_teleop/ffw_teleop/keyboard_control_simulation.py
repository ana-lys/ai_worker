import tkinter as tk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ffw_collision_checker.msg import CollisionCheck
from ffw_collision_checker.srv import SolveCollisionNaive
from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory
from collections import deque
import os
import time


class SliderSimControl(Node):
    def __init__(self):
        super().__init__('slider_sim_control')
        
        # --- Control Parameters ---
        self.Kpos = 15.0           # Position gain (Proportional)
        self.Kvel = 0.2           # Velocity gain (Proportional on velocity error)
        self.v_max = 2.5          # Max velocity limit (rad/s)
        self.position_threshold = 0.005  # Deadband for position error
        self.control_rate_hz = 100.0     # Assumed control loop rate
        self.dt = 1.0 / self.control_rate_hz

        # --- State Variables ---
        self.in_collision = False
        self.setback_needed = False
        self.collision_details = []
        self.last_collision_warn_time = 0.0
        self.first_state_received = False
        self.running = True
        self.collision_server = False
        self.gui_updating = False
        self.collision_recovery = False
        self.collision_recovery_queue = deque()
        self.collision_recovery_active = None
        self.collision_recovery_started = False
        # --- GUI Components ---
        self.sliders = {}
        self.value_labels = {}
        self.root = tk.Tk()
        self.root.title('Slider Simulation Control - Cascaded Controller')

        # --- Robot Configuration ---
        self.joint_limits = self.load_joint_limits_from_urdf()
        self.controllers = self._init_controllers()

        # --- ROS Interfaces ---
        # Joint state subscription (drives the control loop)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Collision check subscription
        self.collision_sub = self.create_subscription(
            CollisionCheck, '/collision/collision_check', self.collision_callback, 10
        )
        
        # Collision solver service server
        self.solve_collision_service = self.create_service(
            SolveCollisionNaive,
            '/collision/solve_collision_naive',
            self.handle_solve_collision
        )

        # Build GUI
        self.build_gui()
        self.get_logger().info('Slider Simulation Control initialized.')

    def _init_controllers(self):
        """Initialize controller configurations"""
        controllers = {
            'arm_l': {
                'joints': [
                    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                    'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
                    'gripper_l_joint1'
                ],
                'topic': '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory'
            },
            'arm_r': {
                'joints': [
                    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                    'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
                    'gripper_r_joint1'
                ],
                'topic': '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory'
            },
            'head': {
                'joints': ['head_joint1', 'head_joint2'],
                'topic': '/leader/joystick_controller_left/joint_trajectory'
            },
            'lift': {
                'joints': ['lift_joint'],
                'topic': '/leader/joystick_controller_right/joint_trajectory'
            }
        }

        # Initialize state vectors and publishers
        for key, ctrl in controllers.items():
            num_joints = len(ctrl['joints'])
            ctrl['current_pos'] = [0.0] * num_joints
            ctrl['current_vel'] = [0.0] * num_joints
            ctrl['target'] = [0.0] * num_joints
            ctrl['commanded'] = [0.0] * num_joints
            ctrl['setback_checkpoint_pos'] = [0.0] * num_joints
            ctrl['setback_checkpoint_vel'] = [0.0] * num_joints
            # Set limits
            ctrl['limits'] = [
                self.joint_limits.get(joint, (-3.14, 3.14))
                for joint in ctrl['joints']
            ]
            
            # Create publisher
            ctrl['publisher'] = self.create_publisher(
                JointTrajectory, ctrl['topic'], 10)
            
            self.get_logger().info(f'{key}: {list(zip(ctrl["joints"], ctrl["limits"]))}')
            
        return controllers

    def run(self):
        """Main GUI loop"""
        self.get_logger().info('Starting GUI loop...')
        try:
            while rclpy.ok() and self.running:
                rclpy.spin_once(self, timeout_sec=0.01)
                self.root.update()
        except tk.TclError:
            self.running = False

    def joint_state_callback(self, msg):
        """Update current positions/velocities and run control loop"""
        # Update current state from feedback
        for ctrl in self.controllers.values():
            for i, joint in enumerate(ctrl['joints']):
                if joint in msg.name:
                    idx = msg.name.index(joint)
                    ctrl['current_pos'][i] = msg.position[idx]
                    ctrl['current_vel'][i] = msg.velocity[idx] if msg.velocity else 0.0
                    
                    # Initialize commanded and target on first feedback
                    if not self.first_state_received:
                        ctrl['commanded'][i] = msg.position[idx]
                        ctrl['target'][i] = msg.position[idx]
        
        if self.setback_needed:
            
            for ctrl in self.controllers.values():
                for i, joint in enumerate(ctrl['joints']):
                    if joint in msg.name:
                        ctrl['target'][i] = ctrl['setback_checkpoint_pos'][i] - 0.5 * ctrl['setback_checkpoint_vel'][i]
                        if joint == "lift_joint":
                            self.get_logger().info(f'Setback due to collision, adjusting targets {joint} {ctrl["target"][i]} {ctrl["current_pos"][i]} {ctrl["setback_checkpoint_vel"][i]}')
            self.control_loop()
            return

        if not self.first_state_received:
            self.first_state_received = True
            self.update_gui_from_state()
            self.get_logger().info('First state received, initialized positions')

        if self.collision_recovery:
            self.process_collision_recovery()
            # Allow control loop even if in_collision while recovering
            self.control_loop()
            return

        # Run control loop driven by joint state updates
        if not self.in_collision and self.collision_server and not self.setback_needed:
            self.control_loop()
            for ctrl in self.controllers.values():
                ctrl["setback_checkpoint_pos"] = list(ctrl['current_pos'])  
                ctrl["setback_checkpoint_vel"] = list(ctrl['current_vel'])
        else:
            self._handle_collision_halt()

    def _handle_collision_halt(self):
        """Log warning when halted due to collision (throttled)"""
        current_time = time.time()
        if current_time - self.last_collision_warn_time > 5.0:
            self.get_logger().warn('Control halted due to collision!')
            self.last_collision_warn_time = current_time

    def control_loop(self):
        """
        Cascaded Position-Velocity Control Loop
        Runs at the rate of joint_state_callback (approx 100Hz)
        
        Control Law:
        1. Position Error: e_pos = target - current
        2. Desired Velocity: v_cmd = Kpos * e_pos (clamped to v_max)
        3. Velocity Correction: v_adjust = Kvel * (v_cmd - v_current)
        4. Next Position: p_next = p_current + (v_cmd + v_adjust) * dt
        """
        if not self.first_state_received:
            return
        
        for ctrl in self.controllers.values():
            new_positions = []
            velocities = []
            needs_update = False
            
            for i in range(len(ctrl['joints'])):
                current_pos = ctrl['current_pos'][i]
                current_vel = ctrl['current_vel'][i]
                target_pos = ctrl['target'][i]
                
                pos_error = target_pos - current_pos
                
                if abs(pos_error) > self.position_threshold:
                    needs_update = True
                    
                    # 1. Calculate desired velocity (P-control on position)
                    v_cmd = self.Kpos * pos_error 
                    
                    # 2. Limit velocity
                    v_cmd = max(-self.v_max, min(self.v_max, v_cmd))
                    
                    # 3. Calculate velocity adjustment (P-control on velocity)
                    # This acts as a feedforward + damping term
                    v_adjust = self.Kvel * (v_cmd - current_vel)
                    
                    # 4. Integrate to get next position command
                    p_new = current_pos + (v_cmd + v_adjust) * self.dt
                    
                    # 5. Clamp to joint limits
                    min_limit, max_limit = ctrl['limits'][i]
                    p_new = max(min_limit, min(max_limit, p_new))
                else:
                    v_cmd = 0.0
                    p_new = current_pos
                
                new_positions.append(p_new)
                velocities.append(v_cmd)
            
            if needs_update:
                self._publish_command(ctrl, new_positions, velocities)

    def _publish_command(self, ctrl, positions, velocities):
        """Publish JointTrajectory command"""
        ctrl['commanded'] = positions
        
        traj = JointTrajectory()
        traj.joint_names = ctrl['joints']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0
        
        traj.points.append(point)
        ctrl['publisher'].publish(traj)

    def collision_callback(self, msg):
        """Handle collision check messages"""
        self.collision_server = True
        
        if  msg.in_collision:
            self.setback_needed = True
            self.in_collision = True
            self.collision_details = []
            for i, dist in enumerate(msg.distances):
                if dist < 0:  # Only store actual collisions (negative distance)
                    self.collision_details.append({
                        'geom1': msg.geom1_names[i],
                        'geom2': msg.geom2_names[i],
                        'distance': dist
                    })
            
            self.update_collision_display()
            self._log_collision()
        else:
            if self.setback_needed:
                self.setback_needed = False
                self.collision_recovery = True
            self._clear_collision_display()

    def _log_collision(self):
        collision_info = ', '.join([
            f"{c['geom1']} <-> {c['geom2']} (dist: {c['distance']:.4f})"
            for c in self.collision_details
        ])
        # self.get_logger().error(f'COLLISION DETECTED: {collision_info}')

    def update_collision_display(self):
        """Update GUI to show collision information"""
        if not hasattr(self, 'collision_label'):
            return
        
        if self.in_collision and self.collision_details:
            collision_text = 'COLLISION DETECTED!\n'
            for c in self.collision_details[:3]:  # Show first 3 collisions
                collision_text += f"{c['geom1']} ↔ {c['geom2']}\n"
                collision_text += f"Penetration: {abs(c['distance']):.4f}m\n"
            
            self.collision_label.config(text=collision_text, bg='red', fg='white')

    def _clear_collision_display(self):
        if hasattr(self, 'collision_label'):
            self.collision_label.config(text='NO COLLISION', bg='green', fg='white')

    def process_collision_recovery(self):
        """Process queued joints, zeroing them one by one during recovery."""
        if not self.collision_recovery :
            return
        if not self.collision_recovery_started:
            return

        tolerance = 0.01
        timeout_per_joint = 10.0
        # If no active joint, take next from queue
        if self.collision_recovery_active is None:
            if not self.collision_recovery_queue:
                # Done
                self.collision_recovery = False
                self.collision_recovery_active = None
                self.collision_recovery_started = False
                self.get_logger().info('Collision recovery completed')
                self.clear_collision()
                return
            ctrl_key, joint_idx, joint_name, zero = self.collision_recovery_queue.popleft()
            ctrl = self.controllers[ctrl_key]
            ctrl['target'][joint_idx] = zero
            if (ctrl_key, joint_idx) in self.sliders:
                self.sliders[(ctrl_key, joint_idx)].set(zero)
            if (ctrl_key, joint_idx) in self.value_labels:
                self.value_labels[(ctrl_key, joint_idx)].config(text=f'{zero:.2f}')
            self.collision_recovery_active = {
                'ctrl_key': ctrl_key,
                'joint_idx': joint_idx,
                'joint_name': joint_name,
                'zero': zero,
                'start': time.time()
            }
            self.get_logger().info(f'Collision recovery: commanding {joint_name} -> {zero:.3f}')
            return

        # Check active joint progress
        active = self.collision_recovery_active
        ctrl = self.controllers[active['ctrl_key']]
        current_pos = ctrl['current_pos'][active['joint_idx']]
        if abs(current_pos - active['zero']) <= tolerance:
            self.get_logger().info(f'Collision recovery: {active["joint_name"]} reached {current_pos:.3f}')
            self.collision_recovery_active = None
            return

        # Timeout guard
        if time.time() - active['start'] > timeout_per_joint:
            self.get_logger().warn(
                f'Collision recovery: timeout waiting for {active["joint_name"]} to reach zero (current {current_pos:.3f})'
            )
            self.collision_recovery_active = None


    def update_gui_from_state(self):
        """Update GUI sliders to match current robot state"""
        self.gui_updating = True
        for ctrl_key, ctrl in self.controllers.items():
            for i in range(len(ctrl['joints'])):
                if (ctrl_key, i) in self.sliders:
                    self.sliders[(ctrl_key, i)].set(ctrl['current_pos'][i])
                if (ctrl_key, i) in self.value_labels:
                    self.value_labels[(ctrl_key, i)].config(text=f'{ctrl["current_pos"][i]:.2f}')
        self.gui_updating = False

    def on_slider_change(self, ctrl_key, joint_idx, value):
        """Update target when slider moves"""
        if self.gui_updating:
            return
        ctrl = self.controllers[ctrl_key]
        ctrl['target'][joint_idx] = float(value)
        if (ctrl_key, joint_idx) in self.value_labels:
            self.value_labels[(ctrl_key, joint_idx)].config(text=f'{float(value):.2f}')

    def build_gui(self):
        """Build the GUI interface"""
        title = tk.Label(
            self.root,
            text='SLIDER SIMULATION CONTROL\nCascaded Controller (~100Hz)',
            font=('Arial', 11, 'bold'),
            fg='white',
            bg='green',
            pady=10
        )
        title.grid(row=0, column=0, columnspan=5, sticky='ew')
        
        # Collision status display
        self.collision_label = tk.Label(
            self.root,
            text='NO COLLISION',
            font=('Arial', 14, 'bold'),
            bg='green',
            fg='white',
            pady=15,
            relief=tk.RAISED,
            borderwidth=3
        )
        self.collision_label.grid(row=1, column=0, columnspan=5, sticky='ew', padx=10, pady=5)
        
        row = 2
        for ctrl_key, ctrl in self.controllers.items():
            # Controller header
            header = tk.Label(
                self.root,
                text=ctrl_key.upper(),
                font=('Arial', 12, 'bold'),
                bg='lightgray',
                pady=5
            )
            header.grid(row=row, column=0, columnspan=5, sticky='ew')
            row += 1
            
            # Joint sliders
            for i, joint in enumerate(ctrl['joints']):
                min_limit, max_limit = ctrl['limits'][i]

                # Joint name
                name_label = tk.Label(self.root, text=joint, width=20, anchor='w')
                name_label.grid(row=row, column=0, padx=5, pady=2, sticky='w')

                # Min limit label
                min_label = tk.Label(self.root, text=f'{min_limit:.2f}', width=6, anchor='e', fg='blue')
                min_label.grid(row=row, column=1, padx=(5,0), pady=2, sticky='e')

                # Slider
                slider = tk.Scale(
                    self.root,
                    from_=min_limit,
                    to=max_limit,
                    resolution=0.01,
                    orient=tk.HORIZONTAL,
                    length=250,
                    showvalue=False,
                    command=lambda val, c=ctrl_key, j=i: self.on_slider_change(c, j, val)
                )
                slider.set(0.0)
                slider.grid(row=row, column=2, padx=0, pady=2)
                self.sliders[(ctrl_key, i)] = slider

                # Max limit label
                max_label = tk.Label(self.root, text=f'{max_limit:.2f}', width=6, anchor='w', fg='red')
                max_label.grid(row=row, column=3, padx=(0,5), pady=2, sticky='w')

                # Current value label
                value_label = tk.Label(self.root, text='0.00', width=8, font=('Courier', 10))
                value_label.grid(row=row, column=4, padx=5, pady=2)
                self.value_labels[(ctrl_key, i)] = value_label

                row += 1
        
        # Control buttons frame
        button_frame = tk.Frame(self.root)
        button_frame.grid(row=row, column=0, columnspan=5, pady=10, sticky='ew')
        
        # Reset button
        reset_btn = tk.Button(
            button_frame,
            text='RESET ALL TO ZERO',
            command=self.reset_all,
            bg='orange',
            font=('Arial', 10, 'bold'),
            pady=5
        )
        reset_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        # Clear collision button
        clear_collision_btn = tk.Button(
            button_frame,
            text='CLEAR COLLISION & RESUME',
            command=self.clear_collision,
            bg='blue',
            fg='white',
            font=('Arial', 10, 'bold'),
            pady=5
        )
        clear_collision_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

    def reset_all(self):
        """Reset all joints to zero (or closest valid position)"""
        self.gui_updating = True
        for ctrl_key, ctrl in self.controllers.items():
            for i in range(len(ctrl['joints'])):
                min_limit, max_limit = ctrl['limits'][i]
                zero = max(min(0.0, max_limit), min_limit)
                ctrl['target'][i] = zero
                if (ctrl_key, i) in self.sliders:
                    self.sliders[(ctrl_key, i)].set(zero)
                if (ctrl_key, i) in self.value_labels:
                    self.value_labels[(ctrl_key, i)].config(text=f'{zero:.2f}')
        self.gui_updating = False
        self.get_logger().info('Reset all joints to zero')

    def clear_collision(self):
        """Clear collision state and resume control"""
        self.in_collision = False
        self.collision_details = []
        self.update_collision_display()
        self.get_logger().info('Collision cleared, resuming control')

    def handle_solve_collision(self, request, response):
        """Handle collision solver service request - queue joints to zero sequentially"""
        self.get_logger().info(f'Received collision solution with {len(request.joint_names)} joints')
        
        if len(request.joint_names) == 0:
            self.get_logger().warn('Empty collision solution received')
            response.accept = False
            response.error = 1
            return response
        
        try:
            # Build recovery queue (ctrl_key, joint_idx, joint_name, zero_target)
            recovery_items = []
            for joint_name in request.joint_names:
                found = False
                for ctrl_key, ctrl in self.controllers.items():
                    if joint_name in ctrl['joints']:
                        joint_idx = ctrl['joints'].index(joint_name)
                        min_limit, max_limit = ctrl['limits'][joint_idx]
                        zero = max(min(0.0, max_limit), min_limit)
                        recovery_items.append((ctrl_key, joint_idx, joint_name, zero))
                        found = True
                        break
                
                if not found:
                    self.get_logger().warn(f'Joint {joint_name} not found in controllers')

            # Load queue and start recovery
            self.collision_recovery_queue = deque(recovery_items)
            self.collision_recovery_active = None
            self.collision_recovery_started = True

            # Allow control loop to run during recovery
            self.collision_details = []
            self.update_collision_display()
            
            response.accept = True
            response.error = 0
            self.get_logger().info(f'✓ Collision recovery queued: {len(recovery_items)} joints')
        except Exception as e:
            self.get_logger().error(f'Failed to apply collision solution: {e}')
            response.accept = False
            response.error = 2
            self.gui_updating = False
        
        return response

    def load_joint_limits_from_urdf(self):
        """Load joint limits from URDF file"""
        limits = {}
        try:
            package_share_directory = get_package_share_directory('ffw_description')
            urdf_file = os.path.join(
                package_share_directory, 
                'urdf', 
                'ffw_sg2_rev1_follower', 
                'ffw_sg2_follower.urdf'
            )
            
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
            
            self.get_logger().info(f'Loaded limits for {len(limits)} joints')
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