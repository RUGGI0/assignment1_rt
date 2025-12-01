# ROS2 Assignment: Multi-Turtle Control and Collision Management

This project implements a coordinated control system for two Turtlesim agents in ROS2.
The system is composed of two custom nodes:

1. `distance_node`: Supervisory logic responsible for collision detection, freeze control, teleportation, and pen management.
2. `ui_node`: Command-line user interface for velocity input and manual turtle control.

Both nodes communicate through topics and services to ensure safe navigation and consistent behaviour in the presence of collisions.

## 1. System Architecture

The ASCII diagram below represents nodes, topics, services, and data flow.

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

                    distance_node publishes /freeze_turtles topic
                                   |
                                   v
                           +----------------+
                           |    ui_node     |
                           |  (Freeze Ctrl) |
                           +----------------+
```

## 2. Node Descriptions

### 2.1 distance_node

The `distance_node` acts as the supervisory control unit. It performs:

- Subscription to `/turtle1/pose` and `/turtle2/pose`
- Detection of turtle–turtle and turtle–wall collisions
- publication of `/freeze_turtles` to temporarily disable UI input
- Automatic pen color change during collisions
- Teleportation back to the last safe pose
- Management of SetPen and Teleport service clients
- Timer-driven update loop running every 50 ms
- ANSI-colored version output at startup

### 2.2 ui_node

The `ui_node` provides a synchronous terminal-based input system. It handles:

- Selection of turtle 1 or turtle 2
- Input of linear and angular velocities
- Execution of commands for a fixed duration of 1 second
- Freezing behaviour via `/freeze_turtles`
- ANSI-colored user prompts and notices
- Publishing commands to `/turtle1/cmd_vel` and `/turtle2/cmd_vel`

## 3. Topics

### Publishers
- `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/turtle2/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/freeze_turtles` (`std_msgs/msg/Bool`)

### Subscribers
- `/turtle1/pose` (`turtlesim/msg/Pose`)
- `/turtle2/pose` (`turtlesim/msg/Pose`)
- `/freeze_turtles` (`std_msgs/msg/Bool`)

## 4. Services

The system uses the following services:

- `/turtleX/set_pen` (`turtlesim/srv/SetPen`)
- `/turtleX/teleport_absolute` (`turtlesim/srv/TeleportAbsolute`)

These services allow pen color switching, pen on/off control, and absolute teleportation.

## 5. Behaviour Summary

- User-issued velocity commands run for exactly one second.
- If a collision occurs:
  - Input is frozen.
  - Both turtles are stopped.
  - Pen colors change to red.
  - After a short delay, pens are disabled.
  - Turtles are teleported to the last valid pose.
  - Pens return to blue and input is re-enabled.
- The UI informs the user when commands are blocked or unblocked.

## 6. Requirements

- ROS2 Jazzy (or compatible)
- turtlesim package installed
- C++17 or later
- Terminal supporting ANSI escape sequences

## 7. Running the Project

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

## 8. Notes on Implementation Style

The codebase follows modern C++ practices:

- Namespaces replaced with short, clear aliases
- Separation of responsibilities between nodes
- Clean structure and readability-focused layout
- Minimal use of fully qualified ROS types
- ANSI-enhanced console readability

## 9. License

This project is distributed under the MIT License.

