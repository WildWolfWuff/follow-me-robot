import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class MovementSimulatorNode(Node):
    def __init__(self):
        super().__init__("movement_simulator")
        self.subscription = self.create_subscription(
            Vector3, "movement_commands", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Movement Simulator Node has been started")

    def listener_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        self.get_logger().info(f"Received: x={x}, y={y}, z={z}")


def main(args=None):
    rclpy.init(args=args)
    node = MovementSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
