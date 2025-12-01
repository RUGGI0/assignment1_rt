import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import time
from datetime import datetime, timedelta

# ANSI colors
Y = "\033[33m"
R = "\033[0m"


# ============================================================
# UI NODE (PYTHON)
# ============================================================
class UINode(Node):
    def __init__(self):
        super().__init__("ui_node")

        self.get_logger().info(f"{Y}ui_node v1.7 loaded{Y}")

        # -------------------------------------------------------
        # Publishers
        # -------------------------------------------------------
        self.pub1 = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pub2 = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        # -------------------------------------------------------
        # Freeze subscriber
        # -------------------------------------------------------
        self.freeze = False
        self.freeze_sub = self.create_subscription(
            Bool,
            "/freeze_turtles",
            self.freeze_callback,
            10
        )

        # -------------------------------------------------------
        # Internal state
        # -------------------------------------------------------
        self.waiting_input = True
        self.selected_turtle = 1
        self.cmd = Twist()
        self.cmd_end_time: datetime | None = None

        # -------------------------------------------------------
        # Timer loop — 100ms
        # -------------------------------------------------------
        self.timer = self.create_timer(0.1, self.loop)

    # ============================================================
    # FREEZE CALLBACK
    # ============================================================
    def freeze_callback(self, msg: Bool):
        """Blocks or unblocks UI input."""
        self.freeze = msg.data

        if self.freeze:
            print(Y + "\n[UI] INPUT BLOCKED BY COLLISION\n" + R)
            self.stop_all()
            self.waiting_input = True
        else:
            print(Y + "\n[UI] INPUT UNBLOCKED\n" + R)

    # ============================================================
    # LOOP
    # ============================================================
    def loop(self):
        """Main loop executed every 100 ms."""
        if self.freeze:
            # UI blocked → no commands
            self.stop_all()
            return

        # ----------------------------------------------
        # Case 1: waiting for input
        # ----------------------------------------------
        if self.waiting_input:
            self.read_user_input()
            self.cmd_end_time = datetime.now() + timedelta(seconds=1)
            self.waiting_input = False
            return

        # ----------------------------------------------
        # Case 2: executing command
        # ----------------------------------------------
        if datetime.now() < self.cmd_end_time:
            self.publish_cmd(self.cmd)
        else:
            self.stop_all()
            print(Y + "\nCommand completed. Ready for next.\n" + R)
            self.waiting_input = True

    # ============================================================
    # USER INPUT
    # ============================================================
    def read_user_input(self):
        """Reads input from keyboard (blocking)."""
        print(Y + "\nSelect turtle to control (1 or 2): " + R, end="")
        try:
            t = int(input())
        except ValueError:
            t = 1

        if t not in (1, 2):
            print(Y + "Invalid input, default = turtle1" + R)
            t = 1

        self.selected_turtle = t

        print(Y + "Linear velocity: " + R, end="")
        try:
            self.cmd.linear.x = float(input())
        except ValueError:
            self.cmd.linear.x = 0.0

        print(Y + "Angular velocity: " + R, end="")
        try:
            self.cmd.angular.z = float(input())
        except ValueError:
            self.cmd.angular.z = 0.0

        print(Y + "Sending command for 1 second..." + R)

    # ============================================================
    # PUBLISH COMMAND
    # ============================================================
    def publish_cmd(self, msg: Twist):
        """Publishes velocity to selected turtle."""
        if self.selected_turtle == 1:
            self.pub1.publish(msg)
        else:
            self.pub2.publish(msg)

    # ============================================================
    # STOP ALL
    # ============================================================
    def stop_all(self):
        stop = Twist()
        self.pub1.publish(stop)
        self.pub2.publish(stop)


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = UINode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
