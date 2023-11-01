"""
Visualizes a turtle robot on rviz that mirrors the position and movement of the
turtle in turtlesim. Concurrently, based on the target positions received,
publishes Twist commands to control the turtlesim and direct it towards said
target positions.

PUBLISHERS:
    joint_states (sensor_msgs/JointState) - Contains the different joint state
        values for the turtle robot in rviz
    turtle1/cmd_vel (geometry_msgs/Twist) - Contains velocity commands to
        direct turtlesim towards its current goal
    odom (nav_msgs/Odometry) - Contains velocity commands that mirror the ones
        being published on cmd_vel

SUBSCRIBERS:
    goal_pose (geometry_msgs/PoseStamped) - Receives the position that the
        turtlesim is to head towards
    tilt (turtle_brick_interfaces/Tilt) - Receives the angle to update the
        joint state of the turtle robot's platform tilt angle
    turtle1/pose (turtlesim/Pose) - Receives the current position of the
        turtlesim

BROADCASTERS:
    world_odom - The static transform from the world to the odom frame
    odom_base_link - The transform from the odom to the base_link frame

LISTENER:
    world_base_link - The transform from the world to the base_link frame of
        the robot
    world_platform_link - The transform from the world to the platform_link
        frame of the robot

PARAMETERS:
    platform_height (double) - The height between the turtle robot's platform
        and the ground
    wheel_radius (double) - The radius of the turtle robot's wheel
    max_velocity (double) - The maximum velocity of the turtle robot
    gravity_accel (double) - The acceleration caused by gravity
    platform_radius (double) - The platform radius of the turtle robot
    frequency (double) - The frequency in which the timer_callback is run
    tolerance (double) - The proximity in which the turtle needs to be
        to a waypoint to classify as arrived

"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist

from enum import Enum


class state(Enum):
    """Current state of the system (turtle_robot node).
    Determines what movement commands are published to the turtle robot,
    whether it is MOVING or STOPPED
    """
    NORMAL = 0
    FAST = 1

class direction(Enum):
    FORWARD = 0
    BACKWARD = 1

class Flip(Node):
    """Central interface between turtlesim and other ROS programs"""

    def __init__(self):
        # Initialize the node
        super().__init__("flip")
        # Initialize variables
        self.init_var()

        #
        # PARAMETERS
        #
        # Declare and get the following parameters: platform_height,
        # wheel_radius, max_velocity, gravity_accel
        self.declare_parameter(
            "count_before_flip",
            300.0,
            ParameterDescriptor(description="The radius of the turtle robot's wheel"),
        )
        self.count_before_flip = (
            self.get_parameter("count_before_flip").get_parameter_value().double_value
        )
        self.declare_parameter(
            "frequency",
            100.0,
            ParameterDescriptor(
                description="The height between the turtle robot's platform and the ground"
            ),
        )
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.declare_parameter(
            "normal_speed",
            0.1,
            ParameterDescriptor(
                description="The height between the turtle robot's platform and the ground"
            ),
        )
        self.normal_speed = (
            self.get_parameter("normal_speed").get_parameter_value().double_value
        )
        self.declare_parameter(
            "fast_speed",
            2.0,
            ParameterDescriptor(description="The radius of the turtle robot's wheel"),
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
        self.state = state.NORMAL

    def timer_callback(self):
        """Timer callback for the flip node.

        Constantly updates the position of the turtle robot in rviz with the posiiton of the
        turtle in turtlesim, additionally controls turtlesim based on node's state and any
        new goal positions it was provided
        """
        # Declaring all the important msgs
        move_msg = Twist()

        # If in NORMAL state, robot moves at its normal, slower speed
        if self.state == state.NORMAL:
            move_msg.linear.x = self.normal_speed
        
        # If in FAST state, robot moves at its faster speed to prepare for flipping
        if self.state == state.FAST:
            move_msg.linear.x = self.fast_speed

        # Flip the robot every self.seconds_before_flip seconds
        # Go slow for the first 3 seconds, then speed up for the remaining time
        # if (self.count % self.count_before_flip) <= (self.count_before_flip - 100):
        #     self.state = state.NORMAL
        # if (self.count % self.count_before_flip) >= (self.count_before_flip - 100):
        #     self.state = state.FAST

        self.state = state.FAST

        # When self.seconds_before_flip seconds is up, flip the robot movement direction
        if round(self.count % self.count_before_flip) == 0:
            self.get_logger().info('Changing direction')
            self.direction_modifier *= -1
        # self.state = state.FAST

        self.get_logger().info('Direction modifier: "%s"' % self.direction_modifier)
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