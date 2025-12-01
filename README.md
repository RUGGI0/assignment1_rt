# ROS2 Assignment: Multi-Turtle Control and Collision Management

This project implements a coordinated control system for two Turtlesim agents in ROS2.  
The system is composed of two custom nodes:

1. `distance_node`: Supervisory logic responsible for collision detection, freeze control, teleportation, and pen management.
2. `ui_node`: Command-line user interface for velocity input and manual turtle control.

Both nodes communicate through topics and services to ensure safe navigation and consistent behaviour in the presence of collisions.


