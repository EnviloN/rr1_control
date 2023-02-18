#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

CONTROLLER_TOPIC = "/joint_trajectory_controller/joint_trajectory"

# https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterType.html
PARAMETER_STRING = rclpy.Parameter.Type.STRING
PARAMETER_DOUBLE_ARRAY = rclpy.Parameter.Type.DOUBLE_ARRAY

class TestJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__('test_joint_trajectory_controller_node')
        self._initialize_parameters()
        
        self.get_logger().info(
            f"Publishing {len(self._positions)} goals on topic '{CONTROLLER_TOPIC}' every {self._timer_period} seconds."
        )

        self._idx = 0
        self._publisher = self.create_publisher(JointTrajectory, CONTROLLER_TOPIC, 1)
        self._timer = self.create_timer(self._timer_period, self._timer_callback)

    def _initialize_parameters(self):
        # Declare all parameters
        self._dynamic_typing = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("timer_period", 6)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("joints", "", self._dynamic_typing)
        self.declare_parameter("check_initial_position", False)
        self.declare_parameter("initial_position_limits", "", self._dynamic_typing)

        # Read parameters
        self._timer_period = self.get_parameter("timer_period").value
        goal_names = self.get_parameter("goal_names").value
        self._joints = self.get_parameter("joints").value
        self._initial_check = self.get_parameter("check_initial_position").value

        if self._joints is None or len(self._joints) == 0:
            raise Exception("Parameter 'joints' is not set!")

        # Initial position check 
        self._handle_initial_position_parameters()
        
        # Read all positions from parameters
        self._read_position_parameters(goal_names)

    def _handle_initial_position_parameters(self):
        self._initial_position_valid = False
        self._joint_state_msg_received = False
        self._initial_position = {}

        if self._initial_check:
            for joint in self._joints:
                parameter_name = "starting_point_limits" + "." + joint
                self.declare_parameter(parameter_name, [-math.tau, math.tau])
                self._initial_position[joint] = self.get_parameter(parameter_name).value

                if len(self._initial_position[joint]) != 2:
                    raise Exception("Parameter 'initial_position_limits' is not set correctly!")

            self._joint_state_subscriber = self.create_subscription(
                JointState, "/joint_states", self._joint_state_callback, 10
            )
        else:
            self._initial_position_valid = True
    
    def _read_position_parameters(self, goal_names):
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

    def _timer_callback(self):
        if self._initial_position_valid:
            trajectory = JointTrajectory()
            trajectory.joint_names = self._joints
            
            point = JointTrajectoryPoint()
            point.positions = self._positions[self._idx]
            point.time_from_start = Duration(sec=4)

            self.get_logger().info(f'Publishing: "{point.positions}"')
            trajectory.points.append(point)
            self._publisher.publish(trajectory)

            self._idx += 1
            self._idx %= len(self._positions)
        elif self._initial_check and not self._joint_state_msg_received:
            self.get_logger().info('Initial configuration has not been checked yet!')
        else:
            self.get_logger().warn("Initial configuration is not within configured limits!")

    def _joint_state_callback(self, msg):
        # Check initial position
        self.get_logger().info(f"Performing check of initial position...")
        limit_exceeded = [False] * len(msg.name)
        for idx, joint in enumerate(msg.name):
            if (msg.position[idx] < self._initial_position[joint][0] or
                msg.position[idx] > self._initial_position[joint][1]):
                self.get_logger().warn(f"Initial position limits exceeded for joint {joint}!")
                limit_exceeded[idx] = True

        self._initial_position_valid = not any(limit_exceeded)
        self._joint_state_msg_received = True
        self.destroy_subscription(self._joint_state_subscriber)


def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS communication

    node = TestJointTrajectoryNode()  # Initialize the node
    rclpy.spin(node)  # Keep the node running until it is killed
    
    rclpy.shutdown()  # Shutdown the ROS communication


if __name__ == '__main__':
    main()
