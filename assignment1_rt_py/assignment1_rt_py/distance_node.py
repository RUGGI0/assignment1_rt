#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from turtlesim.srv import SetPen, TeleportAbsolute

# ANSI colors
Y = "\033[33m"
R = "\033[0m"

# ================================
# CONFIG
# ================================
COLLISION_DISTANCE = 1.5
WALL_THRESHOLD = 1.5
SLEEP_AFTER_COLLISION = 0.4  # seconds


class DistanceNode(Node):

    def __init__(self):
        super().__init__("distance_node")

        self.get_logger().info(f"{Y}distance_node 1.0 loaded{Y}")

        # -----------------------------
        # Internal state
        # -----------------------------
        self.pose1 = None
        self.pose2 = None

        self.last_stop1 = None
        self.last_stop2 = None

        self.init1 = False
        self.init2 = False

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Pose, "/turtle1/pose",
                                 self.pose1_callback, 10)
        self.create_subscription(Pose, "/turtle2/pose",
                                 self.pose2_callback, 10)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.pub1 = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pub2 = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.freeze_pub = self.create_publisher(Bool, "/freeze_turtles", 10)

        # -----------------------------
        # Service Clients
        # -----------------------------
        self.pen1 = self.create_client(SetPen, "/turtle1/set_pen")
        self.pen2 = self.create_client(SetPen, "/turtle2/set_pen")
        self.tp1 = self.create_client(TeleportAbsolute,
                                      "/turtle1/teleport_absolute")
        self.tp2 = self.create_client(TeleportAbsolute,
                                      "/turtle2/teleport_absolute")

        # -----------------------------
        # Timer
        # -----------------------------
        self.create_timer(0.05, self.update)

    # =====================================================
    # UTILITY FUNCTIONS
    # =====================================================

    def freeze(self, state: bool):
        msg = Bool()
        msg.data = state
        self.freeze_pub.publish(msg)

    def stop_turtle(self, pub):
        msg = Twist()
        pub.publish(msg)

    def set_pen(self, client, r, g, b, w, off):
        if not client.wait_for_service(timeout_sec=0.1):
            return

        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = w
        req.off = 1 if off else 0
        client.call_async(req)

    def teleport(self, client, pose):
        if not client.wait_for_service(timeout_sec=0.1):
            return

        req = TeleportAbsolute.Request()
        req.x = pose.x
        req.y = pose.y
        req.theta = pose.theta
        client.call_async(req)

    @staticmethod
    def near_wall(p):
        return (
            p.x < WALL_THRESHOLD
            or p.x > 10.0 - WALL_THRESHOLD
            or p.y < WALL_THRESHOLD
            or p.y > 10.0 - WALL_THRESHOLD
        )

    # =====================================================
    # SUBSCRIBER CALLBACKS
    # =====================================================

    def pose1_callback(self, msg: Pose):
        self.pose1 = msg
        if not self.init1:
            self.init1 = True
            self.last_stop1 = msg
            self.set_pen(self.pen1, 0, 0, 255, 3, False)

        if msg.linear_velocity == 0 and msg.angular_velocity == 0:
            self.last_stop1 = msg

    def pose2_callback(self, msg: Pose):
        self.pose2 = msg
        if not self.init2:
            self.init2 = True
            self.last_stop2 = msg
            self.set_pen(self.pen2, 0, 0, 255, 3, False)

        if msg.linear_velocity == 0 and msg.angular_velocity == 0:
            self.last_stop2 = msg

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def update(self):
        if self.pose1 is None or self.pose2 is None:
            return

        # --------------------------------
        # TURTLE–TURTLE COLLISION
        # --------------------------------
        dx = self.pose1.x - self.pose2.x
        dy = self.pose1.y - self.pose2.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < COLLISION_DISTANCE:
            self.handle_collision("turtles", both=True)
            return

        # --------------------------------
        # WALL COLLISIONS
        # --------------------------------
        if self.near_wall(self.pose1):
            self.handle_collision("turtle1 wall", t1=True)

        if self.near_wall(self.pose2):
            self.handle_collision("turtle2 wall", t2=True)

    # =====================================================
    # COLLISION HANDLING
    # =====================================================

    def handle_collision(self, msg, both=False, t1=False, t2=False):
        self.get_logger().error(f"[COLLISION] {msg}")

        self.freeze(True)

        # Stop turtles
        if both or t1:
            self.stop_turtle(self.pub1)
        if both or t2:
            self.stop_turtle(self.pub2)

        # Flash red
        if both or t1:
            self.set_pen(self.pen1, 255, 0, 0, 3, False)
        if both or t2:
            self.set_pen(self.pen2, 255, 0, 0, 3, False)

        time.sleep(SLEEP_AFTER_COLLISION)

        # Disable pen
        if both or t1:
            self.set_pen(self.pen1, 0, 0, 0, 3, True)
        if both or t2:
            self.set_pen(self.pen2, 0, 0, 0, 3, True)

        # Teleport
        if both or t1:
            self.teleport(self.tp1, self.last_stop1)
        if both or t2:
            self.teleport(self.tp2, self.last_stop2)

        time.sleep(0.05)

        # Re-enable blue pen
        if both or t1:
            self.set_pen(self.pen1, 0, 0, 255, 3, False)
        if both or t2:
            self.set_pen(self.pen2, 0, 0, 255, 3, False)

        # Stop again to avoid drift
        if both or t1:
            self.stop_turtle(self.pub1)
        if both or t2:
            self.stop_turtle(self.pub2)

        self.freeze(False)


# =====================================================
# MAIN
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
