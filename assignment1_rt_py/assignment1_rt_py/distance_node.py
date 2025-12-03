#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from turtlesim.srv import SetPen, TeleportAbsolute

# ANSI
Y = "\033[33m"
R = "\033[0m"

# ================================
# CONFIG
# ================================
PRECOLLISION_DISTANCE = 1.5
COLLISION_DISTANCE = 0.5

WALL_THRESHOLD_MIN = 0.5
WALL_THRESHOLD_MAX = 10.5

SLEEP_AFTER_COLLISION = 0.4  # seconds


class DistanceNode(Node):

    def __init__(self):
        super().__init__("distance_node")

        self.get_logger().info(f"{Y}distance_node 2.3 patched (micro-move) loaded{R}")

        # Internal
        self.pose1 = None
        self.pose2 = None

        self.last_stop1 = None
        self.last_stop2 = None

        self.init1 = False
        self.init2 = False

        self.pre1_active = False
        self.pre2_active = False

        # Subscribers
        self.create_subscription(Pose, "/turtle1/pose", self.pose1_callback, 10)
        self.create_subscription(Pose, "/turtle2/pose", self.pose2_callback, 10)

        # Publishers
        self.pub1 = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pub2 = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.freeze_pub = self.create_publisher(Bool, "/freeze_turtles", 10)

        # Services
        self.pen1 = self.create_client(SetPen, "/turtle1/set_pen")
        self.pen2 = self.create_client(SetPen, "/turtle2/set_pen")
        self.tp1 = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.tp2 = self.create_client(TeleportAbsolute, "/turtle2/teleport_absolute")

        # Timer
        self.create_timer(0.05, self.update)

    # -----------------------------
    # Utility
    # -----------------------------
    def freeze(self, state: bool):
        msg = Bool()
        msg.data = state
        self.freeze_pub.publish(msg)

    def stop_turtle(self, pub):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub.publish(msg)

    def set_pen(self, client, r, g, b, w, off):
        if not client.wait_for_service(timeout_sec=0.1):
            return
        req = SetPen.Request()
        req.r, req.g, req.b = r, g, b
        req.width = w
        req.off = 1 if off else 0
        client.call_async(req)

    def teleport(self, client, pose):
        if pose is None or not client.wait_for_service(timeout_sec=0.1):
            return
        req = TeleportAbsolute.Request()
        req.x = pose.x
        req.y = pose.y
        req.theta = pose.theta
        client.call_async(req)

    @staticmethod
    def near_wall(p: Pose):
        return (
            p.x < WALL_THRESHOLD_MIN
            or p.x > WALL_THRESHOLD_MAX
            or p.y < WALL_THRESHOLD_MIN
            or p.y > WALL_THRESHOLD_MAX
        )

    # -----------------------------
    # Pose callbacks
    # -----------------------------
    def pose1_callback(self, msg):
        self.pose1 = msg
        if not self.init1:
            self.init1 = True
            self.last_stop1 = msg
            self.set_pen(self.pen1, 0, 0, 255, 3, False)
        if msg.linear_velocity == 0.0 and msg.angular_velocity == 0.0:
            self.last_stop1 = msg

    def pose2_callback(self, msg):
        self.pose2 = msg
        if not self.init2:
            self.init2 = True
            self.last_stop2 = msg
            self.set_pen(self.pen2, 0, 0, 255, 3, False)
        if msg.linear_velocity == 0.0 and msg.angular_velocity == 0.0:
            self.last_stop2 = msg

    # -----------------------------
    # Main loop
    # -----------------------------
    def update(self):
        if self.pose1 is None or self.pose2 is None:
            return

        dx = self.pose1.x - self.pose2.x
        dy = self.pose1.y - self.pose2.y
        dist = math.sqrt(dx * dx + dy * dy)

        # Pre-collision pen logic
        self.handle_pre_collision(dist)

        # Collision
        if dist < COLLISION_DISTANCE:
            self.handle_collision(both=True)
            return

        # Wall collisions
        if self.near_wall(self.pose1):
            self.handle_collision(t1=True)
            return

        if self.near_wall(self.pose2):
            self.handle_collision(t2=True)
            return

    # -----------------------------
    # Pre-collision (pen red)
    # -----------------------------
    def handle_pre_collision(self, dist):
        in_pre = dist < PRECOLLISION_DISTANCE

        if in_pre and not self.pre1_active:
            self.set_pen(self.pen1, 255, 0, 0, 3, False)
            self.pre1_active = True
        if not in_pre and self.pre1_active:
            self.set_pen(self.pen1, 0, 0, 255, 3, False)
            self.pre1_active = False

        if in_pre and not self.pre2_active:
            self.set_pen(self.pen2, 255, 0, 0, 3, False)
            self.pre2_active = True
        if not in_pre and self.pre2_active:
            self.set_pen(self.pen2, 0, 0, 255, 3, False)
            self.pre2_active = False

    # -----------------------------
    # Collision handling (patched)
    # -----------------------------
    def handle_collision(self, both=False, t1=False, t2=False):
        self.freeze(True)

        # Select turtles
        use1 = both or t1
        use2 = both or t2

        # Micro movement message
        micro = Twist()
        micro.linear.x = 1.5
        micro.angular.z = 0.0

        # 1. Activate red pen + micro-move
        if use1:
            self.set_pen(self.pen1, 255, 0, 0, 4, False)
            self.pub1.publish(micro)
        if use2:
            self.set_pen(self.pen2, 255, 0, 0, 4, False)
            self.pub2.publish(micro)

        time.sleep(0.12)  # controls minimal dash length

        # 2. Stop & disable pen
        if use1:
            self.stop_turtle(self.pub1)
            self.set_pen(self.pen1, 0, 0, 0, 4, True)

        if use2:
            self.stop_turtle(self.pub2)
            self.set_pen(self.pen2, 0, 0, 0, 4, True)

        # 3. Teleport
        if use1:
            self.teleport(self.tp1, self.last_stop1)
        if use2:
            self.teleport(self.tp2, self.last_stop2)

        time.sleep(0.05)

        # 4. Restore blue
        if use1:
            self.set_pen(self.pen1, 0, 0, 255, 3, False)
        if use2:
            self.set_pen(self.pen2, 0, 0, 255, 3, False)

        self.freeze(False)


# -----------------------------
# Main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

