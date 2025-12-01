# ROS2 Assignment: Multi-Turtle Control and Collision Management

This project implements a coordinated control system for two Turtlesim agents in ROS2.  
The system is composed of two custom nodes:

1. `distance_node`: Supervisory logic responsible for collision detection, freeze control, teleportation, and pen management.  
2. `ui_node`: Command-line user interface for velocity input and manual turtle control.

Both nodes communicate through topics and services to ensure safe navigation and consistent behavior during collisions.

---

## 1. System Architecture

The ASCII diagram below represents nodes, topics, services, and data flow:

```
                           +---------------------------+
                           |         ui_node           |
                           |  (User Command Interface) |
                           +---------------------------+
                                   |         |
                     publishes     |         |    publishes
                     /turtle1/cmd_vel        /turtle2/cmd_vel
                                   |         |
                                   v         v
                     +-----------------+   +-----------------+
                     |   turtle1       |   |    turtle2      |
                     |  (turtlesim)    |   |   (turtlesim)   |
                     +-----------------+   +-----------------+
                             |                      |
                       publishes               publishes
                       /turtle1/pose           /turtle2/pose
                             |                      |
                             v                      v
                    +---------------------------------------+
                    |             distance_node             |
                    |    (Collision and Safety Manager)     |
                    +---------------------------------------+
                             |                      |
            calls SetPenSrv/TeleportSrv     calls SetPenSrv/TeleportSrv
                             |                      |
                             v                      v
                     +------------------+   +------------------+
                     |  turtle1 Pen/TP  |   | turtle2 Pen/TP   |
                     +------------------+   +------------------+

                    distance_node publishes /freeze_turtles
                                   |
                                   v
                           +----------------+
                           |    ui_node     |
                           |  (Freeze Ctrl) |
                           +----------------+
```

---

## 2. Node Descriptions

### 2.1 `distance_node`

`distance_node` functions as the supervisory controller. It performs:

- Subscription to `/turtle1/pose` and `/turtle2/pose`
- Turtle–turtle and turtle–wall collision detection
- Publication of `/freeze_turtles` to temporarily block UI input
- Automatic pen color switching on collisions
- Teleportation to the last safe pose
- Management of SetPen and Teleport service clients
- Timer-driven update loop running at 20 Hz
- ANSI-colored initialization message

---

### 2.2 `ui_node`

`ui_node` provides a synchronous terminal-based interface. It handles:

- Selection between turtle1 and turtle2  
- Input of linear and angular velocities  
- Execution of commands for exactly 1 second  
- Freeze behavior governed by `/freeze_turtles`  
- ANSI-colored user prompts and system messages  

---

## 3. Topics

### Publishers
- `/turtle1/cmd_vel` — `geometry_msgs/msg/Twist`
- `/turtle2/cmd_vel` — `geometry_msgs/msg/Twist`
- `/freeze_turtles` — `std_msgs/msg/Bool`

### Subscribers
- `/turtle1/pose` — `turtlesim/msg/Pose`
- `/turtle2/pose` — `turtlesim/msg/Pose`
- `/freeze_turtles` — `std_msgs/msg/Bool`

---

## 4. Services

- `/turtleX/set_pen` — `turtlesim/srv/SetPen`
- `/turtleX/teleport_absolute` — `turtlesim/srv/TeleportAbsolute`

These allow pen color control and absolute teleportation.

---

## 5. Behavior Summary

- User velocity commands execute for exactly one second.  
- On any collision:  
  - Input freezes  
  - Turtles stop  
  - Pen switches to red  
  - After 400 ms, pen is disabled  
  - Turtles teleport to the last valid pose  
  - Pen switches back to blue  
  - Input is re-enabled  

---

## 6. Requirements

- ROS2 Jazzy (or compatible)
- `turtlesim` package installed
- Terminal with ANSI escape sequence support

---

## 7. Running the Project Manually

Terminal 1:
```
ros2 run turtlesim turtlesim_node
```

Terminal 2:
```
ros2 run assignment1_rt distance_node
```

Terminal 3:
```
ros2 run assignment1_rt ui_node
```

---

## 8. Running the Project via Shell Scripts (Recommended)

Two shell scripts are included to automate execution.

### 8.1 `launcher.sh` — Main Entry Point

`launcher.sh` provides:

- An interactive ASCII-based menu  
- Options to start, restart, cancel, or quit  
- Automatic invocation of `run_assignment.sh`  
- No need to manually run backend scripts  

Run with:
```
chmod +x launcher.sh
./launcher.sh
```

### 8.2 `run_assignment.sh` — Backend Execution Layer

Automatically invoked by `launcher.sh`.  
It:

- Creates a `tmux` session  
- Builds a 2×2 tiled layout  
- Launches:  
  - `turtlesim_node`  
  - Spawn of second turtle  
  - `distance_node`  
  - `ui_node`  

Experts may run it directly, though this is unnecessary:

```
chmod +x run_assignment.sh
./run_assignment.sh
```

---

## 9. License

Distributed under the MIT License.
