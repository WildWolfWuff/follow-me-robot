#!/usr/bin/env python3
import json
import serial
from typing import Optional
import rclpy
from rclpy.node import Node


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller_node")
        self.get_logger().info("Motor Controller Node has been started")

        try:
            self.serial_port = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.send_command()

    def send_command(self):
        # Create a dictionary with the values
        command = {"x": self.x, "y": self.y, "z": self.z}

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
