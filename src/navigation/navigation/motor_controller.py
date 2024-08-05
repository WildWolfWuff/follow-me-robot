#!/usr/bin/env python3
import json
import serial
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.get_logger().info("Motor Controller Node has been started")

        try:
            self.serial_port = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.subscription = self.create_subscription(
            Vector3, "movement_commands", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        self.get_logger().info(f"Received: x={x}, y={y}, z={z}")

        self.send_command(msg)

    def send_command(self, msg):
        command = {"x": msg.x, "y": msg.y, "z": msg.z}
        command_json = json.dumps(command)

        self.serial_port.write(command_json.encode("utf-8"))
        self.get_logger().info(f"Sent command: {command_json}")

    def destroy_node(self):
        zero_command = {"x": 0.0, "y": 0.0, "z": 0.0}
        zero_command_json = json.dumps(zero_command)
        self.serial_port.write(zero_command_json.encode("utf-8"))
        self.get_logger().info(f"Sent zero command: {zero_command_json}")

        super().destroy_node()

def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
