#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

CONTROLLER_TOPIC = "/forward_position_controller/commands"

class ForwardPositionCommandNode(Node):
    def __init__(self, data):
        super().__init__('forward_position_command_node')
        self._publisher = self.create_publisher(Float64MultiArray, CONTROLLER_TOPIC, 10)
        msg = Float64MultiArray()
        msg.data = [float(x) for x in data]
        
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._publisher.publish(msg)

def main(argv):
    rclpy.init(args=None)  # Initialize the ROS communication

    node = ForwardPositionCommandNode(data=argv)  # Initialize the node
    node.destroy_node()
    rclpy.shutdown()  # Shutdown the ROS communication


if __name__ == '__main__':
    print(sys.argv)
    if len(sys.argv) < 2:
        raise RuntimeError("No command was provided as an argument.")
    main(sys.argv[1:])
