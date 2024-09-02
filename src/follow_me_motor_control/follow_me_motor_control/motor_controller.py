#!/usr/bin/env python3
import json
import serial
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.get_logger().info("Motor Controller Node has been started")
        
        self.declare_parameter("serial.port", "/dev/ttyACM0")
        self.declare_parameter("serial.baudrate", 9600)
        self.declare_parameter("serial.timeout", 1)
        self.declare_parameter("topic.name", "movement_commands")

        serialPort = self.get_parameter("serial.port").value
        baudRate = self.get_parameter("serial.baudrate").value
        timeout = self.get_parameter("serial.timeout").value
        topic_name = self.get_parameter("topic.name").value
        
        try:
            self.serial_port = serial.Serial(port=serialPort, baudrate=baudRate, timeout=timeout)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
        
        self.subscription = self.create_subscription(
            Twist, topic_name, self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        self.get_logger().info(f"Received: x={x}, y={y}, z={z}")

        self.send_command(msg)

    def send_command(self, msg):
        command = str(msg.linear.x) + ";" + str(msg.linear.y) + ";" + str(msg.angular.z) + "\n"
        self.serial_port.write(command.encode("utf-8"))
        self.get_logger().info(f"Sent command: {command}")        

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
