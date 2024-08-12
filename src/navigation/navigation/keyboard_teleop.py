import threading
import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.publisher_ = self.create_publisher(Twist, "movement_commands", 10)
        self.msg = Twist()
        self.prev_msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0
        self.get_logger().info("Keyboard Teleop Node has been started")

        self.running = True
        self.key_states = {
            "x": False,
            "w": False,
            "s": False,
            "a": False,
            "d": False,
            "e": False,
            "q": False,
        }
        self.thread = threading.Thread(target=self.keyboard_input_loop)
        self.thread.start()

    def keyboard_input_loop(self):
        curses.wrapper(self.curses_loop)

    def curses_loop(self, stdscr):
        stdscr.nodelay(True)
        while self.running:
            key = stdscr.getch()
            if key == ord("x"):
                self.key_states["x"] = True
            elif key == ord("w"):
                self.key_states["w"] = True
            elif key == ord("s"):
                self.key_states["s"] = True
            elif key == ord("a"):
                self.key_states["a"] = True
            elif key == ord("d"):
                self.key_states["d"] = True
            elif key == ord("e"):
                self.key_states["e"] = True
            elif key == ord("q"):
                self.key_states["q"] = True
            else:
                self.key_states = {k: False for k in self.key_states}

            self.update_message()
            if not self.compare_messages(self.msg, self.prev_msg):
                self.publisher_.publish(self.msg)
                self.get_logger().info(
                    f"Publishing: x={self.msg.linear.x}, y={self.msg.linear.y}, z={self.msg.angular.z}"
                )
                self.prev_msg = self.copy_message(self.msg)

            rclpy.spin_once(self, timeout_sec=0.1)

    def update_message(self):

        if self.key_states["w"]:
            self.msg.linear.x += 0.1
        elif self.key_states["s"]:
            self.msg.linear.x -= 0.1

        if self.key_states["d"]:
            self.msg.linear.y += 0.1
        elif self.key_states["a"]:
            self.msg.linear.y -= 0.1

        if self.key_states["e"]:
            self.msg.angular.z += 0.1
        elif self.key_states["q"]:
            self.msg.angular.z -= 0.1

        if self.key_states["x"]:
            self.msg.linear.x = 0.0
            self.msg.linear.y = 0.0
            self.msg.angular.z = 0.0

        if self.msg.linear.x > 1.0:
            self.msg.linear.x = 1.0
        elif self.msg.linear.x < -1.0:
            self.msg.linear.x = -1.0

        if self.msg.linear.y > 1.0:
            self.msg.linear.y = 1.0
        elif self.msg.linear.y < -1.0:
            self.msg.linear.y = -1.0

        if self.msg.angular.z > 1.0:
            self.msg.angular.z = 1.0
        elif self.msg.angular.z < -1.0:
            self.msg.angular.z = -1.0

    def compare_messages(self, msg1, msg2):
        return msg1.linear.x == msg2.linear.x and msg1.linear.y == msg2.linear.y and msg1.angular.z == msg2.angular.z

    def copy_message(self, msg):
        new_msg = Twist()
        new_msg.linear.x = msg.linear.x
        new_msg.linear.y = msg.linear.y
        new_msg.angular.z = msg.angular.z
        return new_msg

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
