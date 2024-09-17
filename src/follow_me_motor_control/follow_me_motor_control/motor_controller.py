#!/usr/bin/env python3
import json
import serial
from typing import Optional
import rclpy
from rclpy.node import Node, QoSProfile
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
    def __init__(self, node_name):
        super().__init__(node_name)
        # Define node parameters
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.z_prev = 0.0
        self.declare_parameter("serial.port", "/dev/ttyACM0")#  The serial port for communication with the motor controller
        self.declare_parameter("serial.baudrate", 9600) # The baud rate for serial communication
        self.declare_parameter("serial.timeout", 1.0) # The timeout for serial communication
        self.declare_parameter("velocity_topic", "/cmd_vel") # The topic for receiving velocity commands
        self.declare_parameter("velocity_tolerance", 0.05) # The tolerance for linear and angular velocities
        # Get node parameters
        self.tollerance = self.get_parameter("velocity_tolerance").value
        serialPort = self.get_parameter("serial.port").value
        baudRate = self.get_parameter("serial.baudrate").value
        timeout = self.get_parameter("serial.timeout").value
        topic_name = self.get_parameter("velocity_topic").value

        try:
            self.get_logger().info(f"Opening serial port: {serialPort} at {baudRate} baudrate")
            self.serial_port = serial.Serial(port=serialPort, baudrate=baudRate, timeout=timeout)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e.errno}")
            raise e
        
        self.subscription = self.create_subscription(
            Twist, topic_name, self.listener_callback,10
        )
        self.get_logger().info("Motor Controller Node has been started")

    def listener_callback(self, msg):
        # Extract linear and angular velocities from the Twist message
        x = round(msg.linear.x,2)
        y = round(msg.linear.y,2)
        z = round(msg.angular.z,2)
        toll=self.tollerance
        is_diff=abs(self.x_prev - x) > toll or abs(self.y_prev - y) > toll or abs(self.z_prev - z) > toll
        self.x_prev = x
        self.y_prev = y
        self.z_prev = z
        self.get_logger().debug(f"Received: x={x}, y={y}, z={z}") # Log the received command
        if is_diff:
            self._send_command(x,y,z)
        # self.get_logger().debug(f"Received: x={x}, y={y}, z={z}") # Log the received command

    def _send_command(self, x=0.0, y=0.0, z=0.0):
        # Send the movement command to the motor controller
        # The command format is "x;y;z\n" where x, y, and z are the linear and angular velocities
        command = f"{str(x)};{str(y)};{str(z)}\n"
        self.serial_port.write(command.encode("utf-8"))
        self.get_logger().info(f"Sent command: {command}")

    def destroy_node(self):
        # Sends a stop command to the motor controller before shutting down the node
        self._send_command()
        self.subscription.destroy()
        super().destroy_node()

def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = MotorControllerNode("motor_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
