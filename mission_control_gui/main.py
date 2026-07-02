#!/usr/bin/env python3
"""
Step 4: Mission Control GUI with Nav2 dispatch + live telemetry panel.

Adds on top of Step 3:
  - Subscription to /odom for live position, heading, and velocity
  - A telemetry panel in the GUI showing this data, updated via signals
  - An Emergency Stop button that publishes a zero Twist on /cmd_vel

Run with:
    python3 main.py
"""

import sys
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from PyQt5.QtCore import QObject, pyqtSignal, QThread, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QTextEdit, QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox
)


# ---------------------------------------------------------------------------
# Signals: the ONLY channel ROS callbacks use to talk to the GUI thread.
# Never touch a QWidget directly from inside a rclpy callback.
# ---------------------------------------------------------------------------
class MissionSignals(QObject):
    log_message = pyqtSignal(str)          # any status line for the log panel
    waypoint_dispatched = pyqtSignal(int, int)   # (index, total)
    feedback_received = pyqtSignal(float)  # distance remaining
    waypoint_result = pyqtSignal(int, bool)      # (index, succeeded)
    mission_complete = pyqtSignal(int, int)      # (succeeded, total)
    server_unavailable = pyqtSignal()
    goal_rejected = pyqtSignal(int)        # index
    odom_updated = pyqtSignal(float, float, float, float, float)
    # (x, y, yaw_degrees, linear_x, angular_z)
    mission_aborted = pyqtSignal()


# ---------------------------------------------------------------------------
# ROS node: owns the Nav2 action client. Lives on the background QThread.
# ---------------------------------------------------------------------------
class MissionControlNode(Node):
    def __init__(self, signals: MissionSignals):
        super().__init__('mission_control_gui')
        self.signals = signals
        self.waypoints = []
        self.current_index = 0
        self.succeeded = 0
        self._active_goal_handle = None  # tracks the in-flight Nav2 goal, for cancellation
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # --- Telemetry: subscribe to /odom for live position/velocity ---
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # --- Emergency stop: publisher for /cmd_vel ---
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist

        # Convert quaternion z/w to yaw (degrees). Assumes a ground robot
        # with roll/pitch ~0, which holds for TurtleBot on a flat plane.
        yaw_rad = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        )
        yaw_deg = math.degrees(yaw_rad)

        self.signals.odom_updated.emit(
            pos.x, pos.y, yaw_deg, twist.linear.x, twist.angular.z)

    def emergency_stop(self):
        # Publishing zero velocity alone isn't enough -- Nav2's controller
        # server keeps publishing its own commands to /cmd_vel ~20Hz while
        # a goal is active, which instantly overwrites a single zero Twist.
        # We must cancel the active goal so Nav2 itself stops driving.
        if self._active_goal_handle is not None:
            self.signals.log_message.emit('EMERGENCY STOP: cancelling active Nav2 goal...')
            cancel_future = self._active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
        else:
            self.signals.log_message.emit(
                'EMERGENCY STOP: no active goal to cancel, publishing zero velocity anyway.')

        stop_msg = Twist()  # all fields default to 0.0
        self._cmd_vel_pub.publish(stop_msg)
        self.signals.log_message.emit('EMERGENCY STOP: zero velocity published to /cmd_vel')

        # Halt the mission loop -- don't let dispatch_next() send another goal.
        self.current_index = len(self.waypoints)
        self.signals.mission_aborted.emit()

    def _on_cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.signals.log_message.emit('Nav2 confirmed goal cancellation.')
        else:
            self.signals.log_message.emit(
                'Nav2 reported no goals were cancelled (goal may have already finished).')

    def start_mission(self, waypoints):
        self.waypoints = waypoints
        self.current_index = 0
        self.succeeded = 0

        self.signals.log_message.emit('Waiting for Nav2 action server...')
        server_up = self._action_client.wait_for_server(timeout_sec=10.0)
        if not server_up:
            self.signals.server_unavailable.emit()
            self.signals.log_message.emit(
                'ERROR: Nav2 action server not available after 10s.')
            return

        self.dispatch_next()

    def dispatch_next(self):
        if self.current_index >= len(self.waypoints):
            self.signals.mission_complete.emit(self.succeeded, len(self.waypoints))
            self.signals.log_message.emit(
                f'Mission complete: {self.succeeded}/{len(self.waypoints)} waypoints reached')
            return

        x, y = self.waypoints[self.current_index]
        total = len(self.waypoints)
        self.signals.waypoint_dispatched.emit(self.current_index + 1, total)
        self.signals.log_message.emit(
            f'Dispatching waypoint {self.current_index + 1}/{total}: x={x:.2f}, y={y:.2f}')

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.signals.goal_rejected.emit(self.current_index + 1)
            self.signals.log_message.emit(
                f'Waypoint {self.current_index + 1} REJECTED by Nav2')
            self.current_index += 1
            self.dispatch_next()
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.signals.feedback_received.emit(distance)
        self.signals.log_message.emit(f'Feedback: distance remaining = {distance:.2f} m')

    def result_callback(self, future):
        self._active_goal_handle = None
        result = future.result()
        status = result.status
        succeeded = (status == 4)
        if succeeded:
            self.signals.log_message.emit(f'Waypoint {self.current_index + 1} SUCCEEDED')
            self.succeeded += 1
        else:
            self.signals.log_message.emit(
                f'Waypoint {self.current_index + 1} FAILED with status {status}, moving on')
        self.signals.waypoint_result.emit(self.current_index + 1, succeeded)
        self.current_index += 1
        self.dispatch_next()


# ---------------------------------------------------------------------------
# Background thread that spins the rclpy executor.
# ---------------------------------------------------------------------------
class RosThread(QThread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._stop_event = threading.Event()

    def run(self):
        while rclpy.ok() and not self._stop_event.is_set():
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self):
        self._stop_event.set()
        self.executor.shutdown()


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class MainWindow(QMainWindow):
    # Signals used to safely trigger ROS-side calls on the ROS thread
    # from GUI button clicks (cross-thread calls, done via queued signals).
    request_start_mission = pyqtSignal(list)
    request_estop = pyqtSignal()

    def __init__(self, node: MissionControlNode, signals: MissionSignals):
        super().__init__()
        self.node = node
        self.signals = signals

        self.setWindowTitle('Mission Control GUI')
        self.resize(600, 500)

        self.waypoint_fields = []

        central = QWidget()
        main_layout = QVBoxLayout()

        # --- Waypoint input grid ---
        grid = QGridLayout()
        for i in range(3):
            label = QLabel(f'Waypoint {i + 1}:')
            x_label = QLabel('X')
            x_field = QLineEdit()
            x_field.setPlaceholderText('e.g. 2.0')
            y_label = QLabel('Y')
            y_field = QLineEdit()
            y_field.setPlaceholderText('e.g. 1.5')

            grid.addWidget(label, i, 0)
            grid.addWidget(x_label, i, 1)
            grid.addWidget(x_field, i, 2)
            grid.addWidget(y_label, i, 3)
            grid.addWidget(y_field, i, 4)

            self.waypoint_fields.append((x_field, y_field))

        main_layout.addLayout(grid)

        # --- Dispatch button + status ---
        button_row = QHBoxLayout()
        self.status_label = QLabel('Idle')
        self.dispatch_button = QPushButton('Dispatch')
        self.dispatch_button.clicked.connect(self.on_dispatch_clicked)
        button_row.addWidget(self.status_label)
        button_row.addStretch()
        button_row.addWidget(self.dispatch_button)
        main_layout.addLayout(button_row)

        # --- Telemetry panel ---
        telemetry_box = QGroupBox('Live Telemetry (/odom)')
        telemetry_layout = QGridLayout()

        telemetry_layout.addWidget(QLabel('Position X:'), 0, 0)
        self.telem_x_label = QLabel('--')
        telemetry_layout.addWidget(self.telem_x_label, 0, 1)

        telemetry_layout.addWidget(QLabel('Position Y:'), 0, 2)
        self.telem_y_label = QLabel('--')
        telemetry_layout.addWidget(self.telem_y_label, 0, 3)

        telemetry_layout.addWidget(QLabel('Heading (yaw):'), 1, 0)
        self.telem_yaw_label = QLabel('--')
        telemetry_layout.addWidget(self.telem_yaw_label, 1, 1)

        telemetry_layout.addWidget(QLabel('Linear vel:'), 1, 2)
        self.telem_linear_label = QLabel('--')
        telemetry_layout.addWidget(self.telem_linear_label, 1, 3)

        telemetry_layout.addWidget(QLabel('Angular vel:'), 2, 0)
        self.telem_angular_label = QLabel('--')
        telemetry_layout.addWidget(self.telem_angular_label, 2, 1)

        self.estop_button = QPushButton('EMERGENCY STOP')
        self.estop_button.setStyleSheet(
            'background-color: #c0392b; color: white; font-weight: bold;')
        self.estop_button.clicked.connect(self.on_estop_clicked)
        telemetry_layout.addWidget(self.estop_button, 2, 2, 1, 2)

        telemetry_box.setLayout(telemetry_layout)
        main_layout.addWidget(telemetry_box)

        # --- Log panel ---
        log_label = QLabel('Mission Log:')
        main_layout.addWidget(log_label)
        self.log_panel = QTextEdit()
        self.log_panel.setReadOnly(True)
        main_layout.addWidget(self.log_panel)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

        # --- Wire ROS signals to GUI slots ---
        self.signals.log_message.connect(self.log)
        self.signals.waypoint_dispatched.connect(self.on_waypoint_dispatched)
        self.signals.feedback_received.connect(self.on_feedback)
        self.signals.waypoint_result.connect(self.on_waypoint_result)
        self.signals.mission_complete.connect(self.on_mission_complete)
        self.signals.server_unavailable.connect(self.on_server_unavailable)
        self.signals.goal_rejected.connect(self.on_goal_rejected)
        self.signals.odom_updated.connect(self.on_odom_updated)
        self.signals.mission_aborted.connect(self.on_mission_aborted)

        # Cross-thread triggers: connect with queued connections so these
        # actually execute on the ROS thread, not the GUI thread.
        self.request_start_mission.connect(
            self.node.start_mission, type=Qt.QueuedConnection)
        self.request_estop.connect(
            self.node.emergency_stop, type=Qt.QueuedConnection)

    def on_dispatch_clicked(self):
        waypoints = []
        errors = []

        for i, (x_field, y_field) in enumerate(self.waypoint_fields):
            x_text = x_field.text().strip()
            y_text = y_field.text().strip()

            if not x_text or not y_text:
                errors.append(f'Waypoint {i + 1}: X and Y must not be empty.')
                continue

            try:
                x_val = float(x_text)
                y_val = float(y_text)
            except ValueError:
                errors.append(f'Waypoint {i + 1}: X and Y must be numeric.')
                continue

            waypoints.append((x_val, y_val))

        if errors:
            QMessageBox.warning(self, 'Invalid Waypoint Input', '\n'.join(errors))
            self.log('Dispatch aborted due to invalid input.')
            return

        self.log('All waypoints valid. Starting mission...')
        self.dispatch_button.setEnabled(False)
        self.status_label.setText('Mission running...')
        self.request_start_mission.emit(waypoints)

    def on_waypoint_dispatched(self, index, total):
        self.status_label.setText(f'Dispatching waypoint {index}/{total}')

    def on_feedback(self, distance):
        self.status_label.setText(f'Distance remaining: {distance:.2f} m')

    def on_waypoint_result(self, index, succeeded):
        pass  # log_message already covers this; hook for future UI (e.g. checklist)

    def on_mission_complete(self, succeeded, total):
        self.status_label.setText(f'Mission complete: {succeeded}/{total} reached')
        self.dispatch_button.setEnabled(True)

    def on_server_unavailable(self):
        QMessageBox.warning(
            self, 'Nav2 Unavailable',
            'Could not reach the Nav2 action server (/navigate_to_pose).\n'
            'Check that Nav2 is launched and fully active, then try again.')
        self.status_label.setText('Idle')
        self.dispatch_button.setEnabled(True)

    def on_goal_rejected(self, index):
        QMessageBox.warning(self, 'Goal Rejected', f'Waypoint {index} was rejected by Nav2.')

    def on_estop_clicked(self):
        self.request_estop.emit()
        self.log('Emergency stop requested by operator.')

    def on_mission_aborted(self):
        self.status_label.setText('Mission aborted (emergency stop)')
        self.dispatch_button.setEnabled(True)

    def on_odom_updated(self, x, y, yaw_deg, linear_x, angular_z):
        self.telem_x_label.setText(f'{x:.2f} m')
        self.telem_y_label.setText(f'{y:.2f} m')
        self.telem_yaw_label.setText(f'{yaw_deg:.1f} deg')
        self.telem_linear_label.setText(f'{linear_x:.2f} m/s')
        self.telem_angular_label.setText(f'{angular_z:.2f} rad/s')

    def log(self, message: str):
        self.log_panel.append(message)


def main():
    rclpy.init()

    signals = MissionSignals()
    node = MissionControlNode(signals)
    ros_thread = RosThread(node)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node, signals)
    window.show()

    exit_code = app.exec_()

    ros_thread.stop()
    ros_thread.wait(timeout=2000)
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
