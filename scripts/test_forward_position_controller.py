#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class TestForwardCommandNode(Node):
    def __init__(self):
        super().__init__('test_forward_position_controller_node')
        self._initialize_parameters()
        
        self.get_logger().info(
            f'Publishing {len(self._positions)} goals on topic "{self.topic_name}"\
              every {self._timer_period} seconds.'
        )

        self._idx = 0
        self._publisher = self.create_publisher(Float64MultiArray, self.topic_name, 1)
        self._timer = self.create_timer(self._timer_period, self.timer_callback)

    def _initialize_parameters(self):
        # Declare all parameters
        self.declare_parameter("timer_period", 1)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("topic_name", "/rr1/forward_position_controller/commands")

        # Read parameters
        self._timer_period = self.get_parameter("timer_period").value
        goal_names = self.get_parameter("goal_names").value
        self.topic_name = self.get_parameter("topic_name").value

        # Read all positions from parameters
        self._positions = list()
        for name in goal_names:
            self.declare_parameter(name, [0.0, 0.0])
            position = self.get_parameter(name).value
            if position is None or len(position) == 0:
                raise Exception(f'Joint values for goal "{name}" not set!')
            
            joint_positions = []
            for value in position:
                joint_positions.append(float(value))

            self._positions.append(joint_positions)

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self._positions[self._idx]
        
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._publisher.publish(msg)

        self._idx += 1
        self._idx %= len(self._positions)


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication

    node = TestForwardCommandNode()  # Initialize the node
    rclpy.spin(node)  # Keep the node running until it is killed
    
    rclpy.shutdown()  # Shutdown the ROS communication


if __name__ == '__main__':
    main()
