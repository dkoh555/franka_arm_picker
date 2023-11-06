import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK_Response
from rcl_interfaces.msg import ParameterDescriptor
from motion_plan_pkg.move_robot import (
    MoveRobot
)  # This should be "from move_robot import MoveRobot"
from motion_plan_pkg.move_robot import State as MOVEROBOT_STATE
from geometry_msgs.msg import Point, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive

from enum import Enum, auto


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    ADDBOX = (auto(),)  # Add a box to the environment
    REMOVEBOX = (auto(),)  # Remove a box from the environment
    MOVEARM = (auto(),)  # Move the arm to a new position
    GRIPPER = (auto(),)  # Move the gripper to a new configuration
    DONE = auto()  # Do nothing


class PickerNode(Node):
    """Central interface between turtlesim and other ROS programs"""

    def __init__(self):
        super().__init__("picker_node")

        self.declare_parameter(
            "move_group_name",
            "panda_manipulator",
            ParameterDescriptor(description="Name of the move group"),
        )
        self.declare_parameter(
            "ee_frame_id",
            "panda_hand_tcp",
            ParameterDescriptor(description="Name of the e-e frame"),
        )

        self.move_group_name = (
            self.get_parameter("move_group_name").get_parameter_value().string_value
        )

        self.ee_frame_id = (
            self.get_parameter("ee_frame_id").get_parameter_value().string_value
        )

        self.ee_frame_id = "panda_hand_tcp"

        # Fake or Real Mode
        self.declare_parameter(
            "fake_mode",
            True,
            ParameterDescriptor(
                description="Axis that the end effector will rotate around"
            ),
        )
        self.fake_mode = self.get_parameter("fake_mode").get_parameter_value().bool_value
        
        # Position
        self.declare_parameter(
            "goal_x", 0.5, ParameterDescriptor(description="X coordinate goal")
        )
        self.declare_parameter(
            "goal_y", 0.5, ParameterDescriptor(description="Y coordinate goal")
        )
        self.declare_parameter(
            "goal_z", 0.5, ParameterDescriptor(description="Z coordinate goal")
        )
        self.goal_x = self.get_parameter("goal_x").get_parameter_value().double_value
        self.goal_y = self.get_parameter("goal_y").get_parameter_value().double_value
        self.goal_z = self.get_parameter("goal_z").get_parameter_value().double_value

        # Orientation
        self.declare_parameter(
            "theta", 3.141529, ParameterDescriptor(description="Angle of rotation")
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

        self.declare_parameter(
            "keep_pos",
            False,
            ParameterDescriptor(description="If keepting the curremt pose"),
        )
        self.declare_parameter(
            "keep_ori",
            False,
            ParameterDescriptor(description="If keeping the current orientation"),
        )

        self.robot = MoveRobot(self, self.move_group_name, self.fake_mode, self.ee_frame_id)

        # self.keep_pos = self.get_parameter('keep_pos').get_parameter_value().bool_value
        # self.keep_ori = self.get_parameter('keep_ori').get_parameter_value().bool_value

        self.posittion: Point = None
        self.orientation: Quaternion = None

        self.cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            1 / 100, self.timer_callback, callback_group=self.cb_group
        )

        # For sequential Pos commands
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

        self.state = State.ADDBOX
        self.grasp_called = False

    def timer_callback(self):
        self.get_logger().info("Timer callback", once=True)
        # counter allows for only sending 1 goal position
        if self.state == State.MOVEARM:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("In executable code")
                # Define goal point msg
                # point = Point(x=self.goal_x, y=self.goal_y, z=self.goal_z)
                # point.x = self.goal_x
                # point.y = self.goal_y
                # point.z = self.goal_z

                # find orientation in quaternion
                # quat = self.robot.angle_axis_to_quaternion(self.theta, self.rotation_axis)
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
                    # self.comm_count += 1
                    self.robot.grasp()
                elif self.comm_count == 2 and not self.fake_mode:
                    self.get_logger().info("Executing open gripper", once=True)
                    self.state = State.GRIPPER
                    # self.comm_count += 1
                    self.robot.grasp()
                # if self.comm_count == 2:
                #     self.get_logger().info("Executing open gripper", once=True)
                #     pass
                elif self.comm_count <= 2:
                    self.get_logger().info("Executing next command", once=True)
                    self.state = State.MOVEARM
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.get_logger().info(f"{self.robot.state}")
                    # self.comm_count += 1
                else:
                    self.robot.state = MOVEROBOT_STATE.WAITING
                    self.state = State.DONE

        elif self.state == State.GRIPPER:
            self.get_logger().info("Executing gripper command", once=True)
            if self.robot.state == MOVEROBOT_STATE.DONE:
                self.state = State.MOVEARM
                self.robot.state = MOVEROBOT_STATE.WAITING
                self.get_logger().info(f"{self.robot.state}")

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

        elif self.state == State.REMOVEBOX:
            if self.robot.state == MOVEROBOT_STATE.WAITING:
                self.get_logger().info("add box", once=True)

                # Remove a box to environment
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
