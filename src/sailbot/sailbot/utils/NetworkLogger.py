import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from launch import LaunchContext
import logging
import socket
import threading

class NetworkLogger(Node):
    def __init__(self, host, port):
        super().__init__("NetworkLogger")
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(10)
        try:
            self.socket.connect((self.host, self.port))
        except Exception as e:
            self.get_logger().error(F'Failed to connect to {self.host}:{self.port}')
            raise e
        
        self.socket.settimeout(None)
        
        self.logMessages = self.create_subscription(
            Log, "/rosout", self.ROS_LogCallback, 10
        )

        self.get_logger().info(F'Connected to {self.host}:{self.port}')

    def ROS_LogCallback(self, msg):
        self.socket.sendall(str(msg).encode('utf-8'))

def main():
    import os
    DOCKER = os.environ.get("IS_DOCKER", False)
    DOCKER = True if DOCKER == "True" else False

    if DOCKER:
        host = '172.30.100.1'
    else:
        raise Exception("Need to set up host ip for pi")
    
    rclpy.init()
    node = NetworkLogger(host, 1234)
    rclpy.spin(node)
    rclpy.shutdown()