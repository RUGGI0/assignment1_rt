# RT Assignment: Multi-Turtle Control and Collision Management  
### (C++ and Python Implementations)

This project implements a coordinated control system for two *Turtlesim* agents in ROS2.  
It includes both **C++** and **Python** versions of the assignment, each providing the same functionality but implemented in different languages.

The system uses two custom nodes:

1. **distance_node** – Supervisory logic responsible for collision detection, freeze control, teleportation, and pen management.  
2. **ui_node** – A command-line user interface that allows manual control of either turtle for timed velocity commands.

Both nodes communicate using ROS topics and services to ensure safe movement, predictable behaviour in collisions, and responsive control.

---

# 1. System Architecture

Below is a simplified ASCII architecture diagram showing all nodes, topics, and services:

```
                               +---------------------------+
                               |          ui_node          |
                               |   (User Command Input)    |
                               +---------------------------+
                                       |         |
                publishes /turtle1/cmd_vel      publishes /turtle2/cmd_vel
                                       |         |
                                       v         v
                         +-----------------+   +-----------------+
                         |    turtle1      |   |     turtle2     |
                         |   (turtlesim)   |   |    (turtlesim)  |
                         +-----------------+   +-----------------+
                                 |                     |
                           publishes                publishes
                           /turtle1/pose           /turtle2/pose
                                 |                     |
                                 +----------+----------+
                                            |
                                            v
                               +---------------------------+
                               |       distance_node       |
                               | (Collision & Safety Mgr.) |
                               +---------------------------+
                                 |                     |
                    calls SetPen/Teleport     calls SetPen/Teleport
                                 |                     |
                                 v                     v
                           +----------+         +-------------+
                           | turtle1  |         |  turtle2    |
                           | pen/tp   |         |   pen/tp    |
                           +----------+         +-------------+

                               publishes /freeze_turtles
                                            |
                                            v
                                   +----------------+
                                   |    ui_node     |
                                   | (Freeze Ctrl)  |
                                   +----------------+
```

---

# 2. Node Descriptions

## 2.1 `distance_node`
The **supervisory control node**, responsible for:

- Subscribing to `/turtle1/pose` and `/turtle2/pose`
- Detecting turtle–turtle and turtle–wall collisions
- Publishing `/freeze_turtles` to temporarily block user input
- Stopping turtles when needed
- Changing pen colors during collisions
- Teleporting turtles back to the last valid pose
- Managing `SetPen` and `TeleportAbsolute` service clients
- Executing a periodic update loop (50 ms)
- Printing a colored version label at startup

## 2.2 `ui_node`
A **terminal-based user interface**, offering:

- Selection of which turtle to control
- Input of linear and angular velocities
- 1-second timed execution of commands
- Automatic blocking when `/freeze_turtles` is active
- Colored user prompts and status messages
- Publishing velocity commands for each turtle

---

# 3. ROS Interfaces

## 3.1 Topics

### **Publishers**
- `/turtle1/cmd_vel` — `geometry_msgs/msg/Twist`  
- `/turtle2/cmd_vel` — `geometry_msgs/msg/Twist`  
- `/freeze_turtles` — `std_msgs/msg/Bool`  

### **Subscribers**
- `/turtle1/pose` — `turtlesim/msg/Pose`  
- `/turtle2/pose` — `turtlesim/msg/Pose`  
- `/freeze_turtles` — `std_msgs/msg/Bool`  

## 3.2 Services
Used internally for pen manipulation and teleportation:

- `/turtleX/set_pen` — `turtlesim/srv/SetPen`  
- `/turtleX/teleport_absolute` — `turtlesim/srv/TeleportAbsolute`  

---

# 4. Behaviour Overview

The system supports two phases before a collision:

## 1) Normal motion
- Pens are **blue**.
- The user can send velocity commands freely.

## 2) Pre-collision phase
Activated when the distance between the turtles is **below PRECOLLISION_DISTANCE but above COLLISION_DISTANCE**.

During this phase:
- The turtles keep moving normally.
- Their pens turn **red**.
- A **short red trail is drawn**, visually indicating an incoming collision.
- If the turtles move apart again, pens return to **blue**.

## 3) Collision phase
Triggered when:
- The distance ≤ `COLLISION_DISTANCE`, or
- A turtle approaches a wall (x or y outside `[0.5, 10.5]`)

During a collision:
- `/freeze_turtles` is set to **true**
- Movement is blocked
- Pens are turned **off**
- Turtles are teleported back to their last safe stop position
- Pens revert to **blue**
- Input is restored (`/freeze_turtles = false`)

---

# 5. Running the Project Manually

## 5.1 Start Turtlesim  
Open **Terminal 1**:

```bash
source install/setup.bash
ros2 run turtlesim turtlesim_node
```

## 5.2 Spawn Turtle2  
Open **Terminal 2**:

```bash
source install/setup.bash
ros2 service call /spawn turtlesim/srv/Spawn   "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

## 5.3 Start C++ Nodes  
Open **Terminal 3**:

```bash
ros2 run assignment1_rt_cpp distance_node
```

Open **Terminal 4**:

```bash
ros2 run assignment1_rt_cpp ui_node
```

## 5.4 Start Python Nodes  
Instead of the C++ commands above, run:

**Distance node:**
```bash
ros2 run assignment1_rt_py distance_node
```

**UI node:**
```bash
ros2 run assignment1_rt_py ui_node
```

---

# 6. Running the Project with Scripts (aka: the very cool way)

Two automation scripts are provided:

- `launcher.sh` (available in both version) 
- `run_assignment.sh` (available in both version)  

The launchers provide to:

- Start a `tmux` 2×2 session  
- Launch turtlesim  
- Spawn turtle2  
- Start both nodes  
- Handle workspace sourcing  
- Cleanly restart the environment  

Run them with:

```bash
cd ~/ros2_workshop/src/assignment1_rt_cpp
chmod +x run_assignment.sh
chmod +x launcher.sh
./launcher.sh
```
or
```bash
cd ~/ros2_workshop/src/assignment1_rt_py
chmod +x run_assignment.sh
chmod +x launcher.sh
./launcher.sh
```
---

# 7. Terminal Requirements for Script Execution

The launch scripts rely on:

```
x-terminal-emulator
```
This ensures compatibility with terminals like GNOME Terminal, XTerm, etc.

IT'S HIGLY SUGGESTED TO USE "[Terminator Terminal Emulator](https://gnome-terminator.org/)", WICH IS THE ONE PRE-INSTALLED ON UBUNTU MATE (DOCKER ENVIROMENT)

Check if it exists:

```bash
which x-terminal-emulator
```

If missing, install or configure it:

```bash
sudo update-alternatives --config x-terminal-emulator
```

---

# 8. Package Names

This project contains two ROS2 packages:

- **assignment1_rt_cpp** — C++ implementation  
- **assignment1_rt_py** — Python implementation  

Each contains its own `distance_node` and `ui_node`.

---

# 9. License

Distributed under the **MIT License**.
