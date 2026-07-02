# Mission Control GUI

A PyQt5 operator dashboard for Project Kratos's Autonomous Subsystem. It
replaces the headless `mission_client.py` node from Assignment 2 with a
graphical base-station interface: an operator enters up to three waypoints,
dispatches them to Nav2, and watches the mission unfold live — dispatch
status, per-waypoint feedback and results, and real-time telemetry from the
rover, all in one window.

## What it does

- **GUI + ROS 2 node in one process.** The `MainWindow` (PyQt5) and
  `MissionControlNode` (rclpy) run in the same Python process. The ROS node
  spins on a background `QThread`; all communication between the two sides
  goes through Qt signals, so the GUI never freezes and ROS callbacks never
  touch a widget directly.
- **Waypoint entry & validation.** Three labelled X/Y input pairs. On
  Dispatch, every field is checked for emptiness and numeric validity before
  anything is sent to Nav2 — invalid input shows a warning dialog instead of
  dispatching bad coordinates.
- **Sequential Nav2 dispatch.** Waypoints are sent one at a time to
  `/navigate_to_pose` using `send_goal_async` / `get_result_async`. The next
  waypoint is only dispatched after the current one succeeds, is aborted, or
  is rejected — never in parallel, never on a fixed timer.
- **Live feedback and results.** Distance-remaining feedback and
  succeeded/failed results for each waypoint are shown in a scrolling
  mission log and in the status bar.
- **Live telemetry panel.** Subscribes to `/odom` and displays the rover's
  position, heading, and linear/angular velocity, updating continuously.
- **Emergency stop.** Cancels the currently active Nav2 goal and publishes a
  zero-velocity `Twist` to `/cmd_vel`, so the rover actually stops instead of
  having Nav2's own controller immediately overwrite a single zero command.
- **Graceful error handling.** If the Nav2 action server is unreachable, or a
  goal is rejected, the GUI shows a warning dialog instead of crashing or
  hanging silently.

## Dependencies

| Dependency | Version / notes |
|---|---|
| Ubuntu | 22.04 LTS (WSL2) |
| ROS 2 | Humble Hawksbill |
| Python | 3.10 (ships with Ubuntu 22.04) |
| PyQt5 | `pip install PyQt5` (plain pip package, not a ROS dependency) |
| Genesis Physics Engine | 1.1.0+ |
| genesis_ros (ROS bridge) | built and sourced in `~/ros2_ws` |
| genesis_sim (TurtleBot sim) | cloned at `~/genesis_sim` |
| ROS message types used | `nav2_msgs/action/NavigateToPose`, `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/Twist`, `nav_msgs/msg/Odometry` |

## Coordinate file format

Waypoints are entered directly in the GUI (no external file is read at
runtime), as three `X Y` float pairs — the same convention as Assignment 2's
`waypoints.txt`:

```
# x y
2.0 1.5
0.5 3.0
-1.0 2.0
```

## Package layout

```
mission_control_gui/
├── mission_control_gui/
│   ├── __init__.py
│   └── main.py          # GUI + embedded ROS 2 node, single entry point
├── resource/mission_control_gui
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
└── .gitignore
```

## Running the full stack

```bash
# Terminal 1 — Genesis simulator
cd ~/genesis_sim
source ~/ros2_ws/install/setup.bash
python3 turtlebot_sim.py

# Terminal 2 — Nav2 stack (wait for "Managed nodes are active")
cd ~/genesis_sim
source ~/ros2_ws/install/setup.bash
ros2 launch ./launch_nav2.py

# Terminal 3 — Mission Control GUI
source ~/ros2_ws/install/setup.bash
ros2 run mission_control_gui main
```

Verify Nav2 is reachable before dispatching:
```bash
ros2 action list   # should show /navigate_to_pose
```

## Telemetry feature justification

The assignment's telemetry section asks for at least one feature that a real
base-station operator would need, backed by live simulator data. This
project implements a combined **position / heading / velocity** panel
sourced from `/odom`, plus an **emergency stop** control:

- **Position (X, Y)** lets the operator confirm the rover is actually where
  Nav2 believes it is — useful for sanity-checking localization during a
  competition run, not just trusting the dispatch log.
- **Heading (yaw)**, derived from the odometry quaternion, tells the
  operator which way the rover is facing — important context that raw X/Y
  coordinates don't convey, e.g. for anticipating the next turn.
- **Linear / angular velocity** shows whether the rover is actually moving
  or stalled. During testing, this was what first revealed that the
  simulated rover was stuck (near-zero linear velocity while Nav2 reported
  a fixed, unchanging distance-to-goal) — exactly the kind of situation a
  live operator dashboard should surface immediately, rather than the
  operator only realizing something is wrong after a timeout.
- **Emergency stop** is the single most safety-critical control a base
  station can offer. It's implemented to cancel the active Nav2 goal (not
  just publish one zero-velocity command), because Nav2's controller server
  re-publishes to `/cmd_vel` at ~20 Hz while a goal is active — a single
  zero-velocity message gets immediately overwritten unless the goal itself
  is cancelled first.

## Known limitations

- Only three waypoints are supported per mission, per the assignment spec.
- The GUI does not persist waypoints between runs.
- Telemetry is limited to `/odom`; LiDAR and camera panels were considered
  but not implemented in this iteration to keep the dashboard focused and
  well-tested within the assignment timeframe.
