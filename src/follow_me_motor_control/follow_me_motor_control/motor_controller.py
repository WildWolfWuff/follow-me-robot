#!/usr/bin/env python3
import json
import serial
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorControllerNode(Node):
    """
    MotorControllerNode class represents a ROS node for controlling the motors of a robot.
    The node receives movement commands and sends them to the motor controller via serial communication.
    Args:
        Node: The base class for creating a ROS node.
    Attributes:
        serial_port (Serial): The serial connection for communication with the motor controller.
        subscription (Subscription): The subscription for receiving movement commands.
    Methods:
        __init__(): Initializes the MotorControllerNode.
        listener_callback(msg): Callback function for processing movement commands.
        send_command(msg): Sends the movement command to the motor controller.
        destroy_node(): Cleans up resources and shuts down the node.
    """
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
        # Extract linear and angular velocities from the Twist message
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        self.get_logger().debug(f"Received: x={x}, y={y}, z={z}") # Log the received command
        self.send_command(msg)

    def send_command(self, msg):
        # Send the movement command to the motor controller
        # The command format is "x;y;z\n" where x, y, and z are the linear and angular velocities
        command = str(msg.linear.x) + ";" + str(msg.linear.y) + ";" + str(msg.angular.z) + "\n"
        self.serial_port.write(command.encode("utf-8"))
        self.get_logger().debug(f"Sent command: {command}")

    def destroy_node(self):
        # Sends a stop command to the motor controller before shutting down the node
        zero_command = "0.0;0.0;0.0"
        self.serial_port.write(zero_command.encode("utf-8"))
        self.get_logger().info(f"Sent zero command: {zero_command}")
        super().destroy_node()

def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
