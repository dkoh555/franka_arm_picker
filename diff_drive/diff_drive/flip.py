"""
Publishes velocity commands to a differential drive robot on Gazebo that utilizes its physics
to have it flip back and forth.

PUBLISHERS:
    cmd_vel (geometry_msgs/Twist) - Contains velocity commands to
        direct the differential drive robot

PARAMETERS:
    count_before_flip (double) - Number of counts the node goes up to before flipping the robot
    frequency (double) - The frequency of the node timer
    fast_speed (double) - The velocity of the robot moving back and forth

"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist


class Flip(Node):
    """Controls the differential robot, ensuring it goes in the correct
    velocity at the right time to periodically flip
    """
    def __init__(self):
        # Initialize the node
        super().__init__("flip")
        # Initialize variables
        self.init_var()

        #
        # PARAMETERS
        #
        self.declare_parameter(
            "count_before_flip",
            300.0,
            ParameterDescriptor(
                description="Number of counts the node goes up to before flipping the robot"
            ),
        )
        self.count_before_flip = (
            self.get_parameter("count_before_flip").get_parameter_value().double_value
        )
        self.declare_parameter(
            "frequency",
            100.0,
            ParameterDescriptor(description="The frequency of the node timer"),
        )
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.declare_parameter(
            "fast_speed",
            2.0,
            ParameterDescriptor(
                description="The velocity of the robot moving back and forth"
            ),
        )
        self.fast_speed = (
            self.get_parameter("fast_speed").get_parameter_value().double_value
        )

        #
        # PUBLISHERS
        #
        # Create publisher to move the robot
        self.pub_cmdvel = self.create_publisher(Twist, "cmd_vel", 10)

        #
        # TIMER
        #
        # Adjusted frequency for whatever the frequency param value is
        timer_period = 1.0 / self.frequency  # seconds
        # create timer and timer callback
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_var(self):
        """Initialize all of the flip node's variables"""
        self.count = 0
        self.direction_modifier = -1

    def timer_callback(self):
        """Timer callback for the flip node.

        Publishes the current velocity to the differential drive robot, keeping track of when to
        flip it and actually flipping it at the appropriate time
        """
        # Declaring all the important msgs
        move_msg = Twist()
        move_msg.linear.x = self.fast_speed

        # When self.seconds_before_flip seconds is up, flip the robot movement direction
        if round(self.count % self.count_before_flip) == 0:
            self.get_logger().info("Changing direction")
            self.direction_modifier *= -1

        move_msg.linear.x *= self.direction_modifier
        self.pub_cmdvel.publish(move_msg)

        self.count += 1
        if self.count > self.count_before_flip:
            self.count = 1


def flip_entry(args=None):
    rclpy.init(args=args)
    node = Flip()
    rclpy.spin(node)
    rclpy.shutdown()
