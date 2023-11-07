"""
Node that controls Franka Panda arm to pickup and drop object.

Utilizes the custom motion_plan_pkg to move a virtual or real Franka Panda Arm
to pick up an object, before moving it to the side and dropping it.

PARAMETERS:
    move_group_name (string) - Name of the robot arm move group
    ee_frame_id (string) - Name of the robot arm end-effector frame
    fake_mode (bool) - The maximum velocity of the turtle robot
    goal_x (double) - X coordinate goal for pickup
    goal_y (double) - Y coordinate goal for pickup
    goal_z (double) - Z coordinate goal for pickup
    tolerance (double) - The proximity in which the turtle needs to be
        to a waypoint to classify as arrived
    theta (double) - The platform radius of the turtle robot
    rotation_axis (double_array) - The platform radius of the turtle robot

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import ParameterDescriptor
from motion_plan_pkg.move_robot import (
    MoveRobot,
)
from motion_plan_pkg.move_robot import State as MOVEROBOT_STATE
from geometry_msgs.msg import Point, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive

from enum import Enum, auto


class State(Enum):
    """
    Current state of the arm controller node.

    Determines what actions the timer executes at a given moement
    """

    ADDBOX = (auto(),)  # Add a box to the environment
    REMOVEBOX = (auto(),)  # Remove a box from the environment
    MOVEARM = (auto(),)  # Move the arm to a new position
    GRIPPER = (auto(),)  # Move the gripper to a new configuration
    DONE = auto()  # Do nothing


class PickerNode(Node):
    """Node that utilizes the MoveRobot class to control the Franka Panda arm.

    Its goal is to reach a given/default position and pick up an object with its
    gripper, before moving it to another position to drop it
    """

    def __init__(self):
        super().__init__("picker_node")

        #
        # Parameters
        #
        # Required MoveGroup Parameters
        self.declare_parameter(
            "move_group_name",
            "panda_manipulator",
            ParameterDescriptor(description="Name of the move group"),
        )
        self.move_group_name = (
            self.get_parameter("move_group_name").get_parameter_value().string_value
        )

        self.declare_parameter(
            "ee_frame_id",
            "panda_hand_tcp",
            ParameterDescriptor(description="Name of the end-effector frame"),
        )
        self.ee_frame_id = (
            self.get_parameter("ee_frame_id").get_parameter_value().string_value
        )

        # Real or Virtual Arm Parameter
        self.declare_parameter(
            "fake_mode",
            True,
            ParameterDescriptor(
                description="Whether this node will be controlling a virtual Franka Panda arm, or a real one"
            ),
        )
        self.fake_mode = (
            self.get_parameter("fake_mode").get_parameter_value().bool_value
        )

        # Position Parameters
        self.declare_parameter(
            "goal_x",
            0.5,
            ParameterDescriptor(description="X coordinate goal for pickup"),
        )
        self.declare_parameter(
            "goal_y",
            0.0,
            ParameterDescriptor(description="Y coordinate goal for pickup"),
        )
        self.declare_parameter(
            "goal_z",
            0.05,
            ParameterDescriptor(description="Z coordinate goal for pickup"),
        )
        self.goal_x = self.get_parameter("goal_x").get_parameter_value().double_value
        self.goal_y = self.get_parameter("goal_y").get_parameter_value().double_value
        self.goal_z = self.get_parameter("goal_z").get_parameter_value().double_value

        # Orientation Parameters
        self.declare_parameter(
            "theta",
            3.141529,
            ParameterDescriptor(description="Magnitude of angle of rotation"),
        )
        self.declare_parameter(
            "rotation_axis",
            [1.0, 0.0, 0.0],
            ParameterDescriptor(
                description="Axis that the end effector will rotate around"
            ),
        )
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.rotation_axis = (
            self.get_parameter("rotation_axis").get_parameter_value().double_array_value
        )

        # Initialize MoveRobot object
        self.robot = MoveRobot(
            self, self.move_group_name, self.fake_mode, self.ee_frame_id
        )

        # Preparing node timer
        self.cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            1 / 100, self.timer_callback, callback_group=self.cb_group
        )

        # Sequential Franka Panda arm and gripper commands to cycle through
        self.comm_count = 0
        self.pos_list = [
            Point(x=self.goal_x, y=self.goal_y, z=self.goal_z),
            Point(x=0.2, y=0.4, z=0.2),
            Point(x=0.5, y=0.0, z=0.4),
        ]
        self.ori_list = [
            self.robot.angle_axis_to_quaternion(self.theta, self.rotation_axis),
            Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
            Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        ]

        # Initializing node states and variables
        self.state = State.ADDBOX
        self.grasp_called = False

    def timer_callback(self):
        """Timer callback for the picker node.

        Executes actions based on the current state of the node, whilst
        controlling when to switch from one state to the other.
        """
        self.get_logger().info("Timer callback", once=True)
        # counter allows for only sending 1 goal position

        #
        # MOVEARM state
        #
        if self.state == State.MOVEARM:
            # Keep checking the MoveRobot state to view the status of the
            # Franka Panda arm, such as whether it is still moving or waiting for instructions
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("In executable code")
                self.get_logger().info("Publishing command no.%s" % self.comm_count)
                self.robot.find_and_execute(
                    point=self.pos_list[self.comm_count],
                    quat=self.ori_list[self.comm_count],
                )
            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.get_logger().info("Done moving arm")
                self.comm_count += 1
                self.get_logger().info("comm_count:%s" % self.comm_count)
                if self.comm_count == 1 and not self.fake_mode:
                    self.get_logger().info("Executing close gripper", once=True)
                    self.state = State.GRIPPER
                    self.robot.grasp()
                elif self.comm_count == 2 and not self.fake_mode:
                    self.get_logger().info("Executing open gripper", once=True)
                    self.state = State.GRIPPER
                    self.robot.grasp()
                elif self.comm_count <= 2:
                    self.get_logger().info("Executing next command", once=True)
                    self.state = State.MOVEARM
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.get_logger().info(f"{self.robot.state}")
                else:
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.state = State.DONE

        #
        # Controlling the GRIPPER state
        #
        elif self.state == State.GRIPPER:
            self.get_logger().info("Executing gripper command", once=True)
            if self.robot.state == MOVEROBOT_STATE.DONE:
                self.state = State.MOVEARM
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.get_logger().info(f"{self.robot.state}")

        #
        # ADDBOX state
        #
        elif self.state == State.ADDBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)

                # Add a box to environment
                name = "box_0"
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = -0.2
                size = [2.5, 2.5, 0.2]
                shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
                self.robot.add_box(name=name, pose=pose, shape=shape)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.MOVEARM

        #
        # REMOVEBOX state
        #
        elif self.state == State.REMOVEBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)

                # Remove a box from environment
                name = "box_0"
                self.robot.remove_box(name=name)

            elif self.robot.state == MOVEROBOT_STATE.DONE:
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.state = State.DONE

            elif self.state == State.GRASP:
                if self.robot.state == MOVEROBOT_STATE.WAITING:
                    if not self.grasp_called:
                        self.robot.grasp()
                        self.grasp_called = True
                elif self.robot.state == MOVEROBOT_STATE.DONE:
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.state = State.DONE
                    self.grasp_called = False


def picker_entry(args=None):
    rclpy.init(args=args)
    node = PickerNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()
