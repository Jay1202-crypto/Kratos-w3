#!/usr/bin/env python3
"""
Step 3: Mission Control GUI with real Nav2 dispatch.

Combines:
  - Step 1's threading pattern (rclpy executor on a background QThread,
    Qt signals crossing to the GUI thread)
  - Step 2's waypoint input UI + validation
  - Assignment 2's mission_client.py Nav2 action client logic,
    ported into signal-emitting callbacks instead of get_logger() calls

Run with:
    python3 main.py
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from PyQt5.QtCore import QObject, pyqtSignal, QThread, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QTextEdit, QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox
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
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

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
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.signals.feedback_received.emit(distance)
        self.signals.log_message.emit(f'Feedback: distance remaining = {distance:.2f} m')

    def result_callback(self, future):
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
    # Signal used to safely trigger start_mission() on the ROS thread
    # from a GUI button click (cross-thread call, done via queued signal).
    request_start_mission = pyqtSignal(list)

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

        # Cross-thread trigger: connect with a queued connection so
        # start_mission() actually executes on the ROS thread, not the GUI thread.
        self.request_start_mission.connect(
            self.node.start_mission, type=Qt.QueuedConnection)

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
