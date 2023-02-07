#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from rr1_interfaces.srv import AnglesConverter


class AnglesConverterNode(Node):
    """Node providing two services for conversion between radians and degrees."""
    def __init__(self):
        super().__init__("angles_converter")

        self.server_ = self.create_service(
            AnglesConverter, "radians_to_degrees", self.callback_rad_to_deg)
        
        self.server_ = self.create_service(
            AnglesConverter, "degrees_to_radians", self.callback_deg_to_rad)
        
        self.get_logger().info("Angles converter node started.")
    
    def callback_rad_to_deg(self, request, response):
        """Runs when the radians_to_degrees service has been called."""
        response.base = int(((request.base+(math.pi/2))*180)/math.pi)
        response.shoulder = 180-int(((request.shoulder+(math.pi/2))*180)/math.pi)
        response.elbow = int(((request.elbow+(math.pi/2))*180)/math.pi)
        response.gripper = int(((-request.gripper)*180)/(math.pi/2))

        self.get_logger().info("Converted joint angles from radians to degrees.")
        return response
    
    def callback_deg_to_rad(self, request, response):
        """Runs when the degrees_to_radians service has been called."""
        response.base = ((math.pi*request.base) - ((math.pi/2)*180))/180
        response.shoulder = (((180-request.shoulder)*math.pi)-((math.pi/2)*180))/180
        response.elbow = ((math.pi*request.elbow) - ((math.pi/2)*180))/180
        response.gripper = -((math.pi/2)*request.gripper)/180

        self.get_logger().info("Converted joint angles from radians to degrees.")
        return response


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication

    node = AnglesConverterNode()  # Initialize the node
    rclpy.spin(node)  # Keep the node running until it is killed
    
    rclpy.shutdown()  # Shutdown the ROS communication

if __name__ == "__main__":
    main()
