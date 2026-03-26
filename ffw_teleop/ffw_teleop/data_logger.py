import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from threading import Lock
import csv
import os

# --- Message Types ---
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# --- Configuration ---
LOGGING_RATE_HZ = 200.0
TIMER_PERIOD_SEC = 1.0 / LOGGING_RATE_HZ
CSV_FILE_PATH = 'robot_data_log.csv'
MAX_EMPTY_WRITES_BEFORE_STOP = 3  # Stop after 3 consecutive writes with no new data

JOINT_STATE_TOPIC = '/joint_states'

TRAJECTORY_TOPICS = [
    '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
    '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
    '/leader/joystick_controller_left/joint_trajectory',
    '/leader/joystick_controller_right/joint_trajectory'
]

LOGGED_JOINTS = [
    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
    'gripper_l_joint1', 'gripper_r_joint1',
    'head_joint1', 'head_joint2',
    'lift_joint'
]

def joint_nice_name(joint_name: str) -> str:
    if joint_name.startswith('arm_l_joint'):
        return f'left_{joint_name[-1]}'
    if joint_name.startswith('arm_r_joint'):
        return f'right_{joint_name[-1]}'
    if joint_name == 'gripper_l_joint1':
        return 'left_gripper'
    if joint_name == 'gripper_r_joint1':
        return 'right_gripper'
    if joint_name == 'head_joint1':
        return 'head_1'
    if joint_name == 'head_joint2':
        return 'head_2'
    if joint_name == 'lift_joint':
        return 'lift'
    return joint_name


class MultiTopicDataLogger(Node):
    def __init__(self):
        super().__init__('multi_topic_data_logger')
        self.get_logger().info('ROS 2 Multi-Topic Data Logger initialized — waiting for first command...')

        # --- Thread Safety ---
        self.data_lock = Lock()

        # --- Data Storage ---
        self.latest_joint_state = None
        self.joint_data_map = {joint: {'position': 0.0, 'velocity': 0.0} for joint in LOGGED_JOINTS}
        self.joint_commands = {joint: 0.0 for joint in LOGGED_JOINTS}

        # --- Recording Control ---
        self.has_received_command = False
        self.recording_active = False
        self.data_updated = False                  # Reset each timer, set by any callback
        self.empty_write_count = 0                  # Counts consecutive writes with no update

        # --- QoS ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Callback Groups ---
        self.js_group = MutuallyExclusiveCallbackGroup()
        self.traj_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # --- Subscribers ---
        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            qos_profile,
            callback_group=self.js_group
        )

        for topic in TRAJECTORY_TOPICS:
            self.create_subscription(
                JointTrajectory,
                topic,
                lambda msg, t=topic: self.joint_trajectory_callback(msg, t),
                qos_profile,
                callback_group=self.traj_group
            )

        # --- Timer ---
        self.create_timer(TIMER_PERIOD_SEC, self.timer_callback, callback_group=self.timer_group)

        # --- CSV ---
        self._initialize_csv()

    def _mark_data_updated(self):
        """Called by any message callback to signal fresh data"""
        with self.data_lock:
            self.data_updated = True

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        with self.data_lock:
            for joint_name in LOGGED_JOINTS:
                if joint_name in name_to_index:
                    idx = name_to_index[joint_name]
                    pos = msg.position[idx] if idx < len(msg.position) else 0.0
                    vel = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
                    self.joint_data_map[joint_name]['position'] = pos
                    self.joint_data_map[joint_name]['velocity'] = vel

            self._mark_data_updated()  # Fresh state data

    def joint_trajectory_callback(self, msg, topic_name):
        with self.data_lock:
            if not msg.points:
                return

            point = msg.points[0]
            name_to_index = {name: i for i, name in enumerate(msg.joint_names)}

            if not self.has_received_command:
                self.has_received_command = True
                self.recording_active = True
                self.get_logger().info("First command received — STARTING data recording!")

            for joint_name in LOGGED_JOINTS:
                if joint_name in name_to_index:
                    idx = name_to_index[joint_name]
                    if idx < len(point.positions):
                        self.joint_commands[joint_name] = point.positions[idx]

            self._mark_data_updated()  # Fresh command data

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        with self.data_lock:
            # Wait for first command
            if not self.has_received_command:
                return

            # Reset update flag at start of each write cycle
            was_updated = self.data_updated
            self.data_updated = False

            # Check for inactivity
            if not was_updated:
                self.empty_write_count += 1
                if self.empty_write_count >= MAX_EMPTY_WRITES_BEFORE_STOP:
                    if self.recording_active:
                        self.get_logger().info(f"No new data for {MAX_EMPTY_WRITES_BEFORE_STOP} consecutive writes — ENDING recording session.")
                        self.recording_active = False
                        rclpy.try_shutdown()
                    return
            else:
                # Data arrived since last write → reset counter
                self.empty_write_count = 0

            # Normal logging
            js_ts = (self.latest_joint_state.header.stamp.sec +
                     self.latest_joint_state.header.stamp.nanosec / 1e9
                     if self.latest_joint_state else 0.0)

            row = [current_time, js_ts]
            for joint in LOGGED_JOINTS:
                row.append(self.joint_data_map[joint]['position'])
                row.append(self.joint_data_map[joint]['velocity'])
                row.append(self.joint_commands[joint])

            try:
                self.csv_writer.writerow(row)
            except Exception as e:
                self.get_logger().error(f"Error writing to CSV: {e}")

    def _initialize_csv(self):
        file_exists = os.path.isfile(CSV_FILE_PATH)

        header = ['log_time', 'js_timestamp']
        for joint in LOGGED_JOINTS:
            nice = joint_nice_name(joint)
            header.append(f'{nice}_position')
            header.append(f'{nice}_velocity')
            header.append(f'{nice}_cmd')

        try:
            self.csv_file = open(CSV_FILE_PATH, mode='a', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            if not file_exists or os.path.getsize(CSV_FILE_PATH) == 0:
                self.csv_writer.writerow(header)
                self.get_logger().info(f"CSV created with {len(header)} columns.")
            else:
                self.get_logger().info(f"Appending to existing CSV ({len(header)} columns).")
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV file: {e}")
            rclpy.try_shutdown()

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    logger_node = MultiTopicDataLogger()

    executor = MultiThreadedExecutor()
    executor.add_node(logger_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        logger_node.get_logger().info("Shutdown by user (Ctrl+C)")
    finally:
        logger_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()