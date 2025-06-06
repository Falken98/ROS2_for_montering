#!/usr/bin/env python3
# BASED ON: https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85#latest
# ROS/Python2 port by felixvd
# Messing around, SID and Hand-E testing by MOJO
# ROS2 port by Falken98
# This file is part of the ros2_robotiq_urcap_control package.

import socket
import threading
import json
import time
import rclpy
from rclpy.node import Node
import sys
from enum import Enum
from messages.msg import Robotiq2FGripperRobotInput as InputMsg
from messages.msg import Robotiq2FGripperRobotOutput as OutputMsg
from std_msgs.msg import Int16


class RobotiqGripper:
    """
    Communicates with the gripper directly, via socket with string commands, leveraging string names for variables.
    """
    # WRITE VARIABLES (CAN ALSO READ)
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
    ATR = 'ATR'  # atr : auto-release (emergency slow move)
    ADR = 'ADR'  # adr : auto-release direction (open(1) or close(0) during auto-release)
    FOR = 'FOR'  # for : force (0-255)
    SPE = 'SPE'  # spe : speed (0-255)
    POS = 'POS'  # pos : position (0-255), 0 = open
    SID = 'SID'  # sid : gripper id. See Polyscope->Installation->URCaps->Gripper->Dashboard
    # READ VARIABLES
    STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
    PRE = 'PRE'  # position request (echo of last commanded position)
    OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)

    ENCODING = 'UTF-8'  # ASCII and UTF-8 both seem to work

    class GripperStatus(Enum):
        """Gripper status reported by the gripper. The integer values have to match what the gripper sends."""
        RESET = 0
        ACTIVATING = 1
        # UNUSED = 2  # This value is currently not used by the gripper firmware
        ACTIVE = 3

    class ObjectStatus(Enum):
        """Object status reported by the gripper. The integer values have to match what the gripper sends."""
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def __init__(self, robot_ip, gripper_sid=None):
        self.robot_ip = robot_ip
        self._sid_from_input = gripper_sid
        self.sid_used = None
        self._sid_from_get = None
        self._robotiq_urcap_port = 63352
        self.socket = None
        self.command_lock = threading.Lock()

        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

    def connect(self, socket_timeout=2.0):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.robot_ip, self._robotiq_urcap_port))
        self.socket.settimeout(socket_timeout)

        self._sid_from_get = self._get_var(self.SID)
        self._check_sid()
        print(("Setting SID to: " + str(self.sid_used)))
        self._set_vars(var_dict={self.SID: self.sid_used})

    def disconnect(self):
        """Closes the connection with the gripper."""
        self.socket.close()

    def _check_sid(self):
        print(("SID from input: " + str(self._sid_from_input)))
        print(("SID from GET: " + str(self._sid_from_get)))
        if self._sid_from_input is not None:
            if self._sid_from_input != self._sid_from_get:
                raise ValueError("SID don`t match. Check your Polyscope for gripper id.")
        self.sid_used = self._sid_from_get
        print(("SID used: " + str(self.sid_used)))

    def _get_var(self, variable):
        with self.command_lock:
            cmd = "GET " + str(variable) + "\n"
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)

        # expect data of the form 'VAR x', where VAR is an echo of the variable name, and X the value
        # note some special variables (like FLT) may send 2 bytes, instead of an integer. We assume integer here
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError("Unexpected response " +
                             str(data) +
                             " does not match " +
                             str(variable))

        if variable == 'SID':
            value = json.loads(value_str)
            if len(value) == 1:
                value = value.pop()
            else:
                raise ValueError("Unexpected number of SID`s, expected one, got: " +
                                 str(len(value)) +
                                 " " +
                                 str(value_str))
        else:
            value = int(value_str)
        return value

    def _set_vars(self, var_dict):

        cmd = "SET"
        for variable, value in list(var_dict.items()):
            cmd += " " + str(variable) + " " + str(value)
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    @staticmethod
    def _is_ack(data):
        return data == b'ack'

    def activate(self, auto_calibrate=True):
        """Resets the activation flag in the gripper, and sets it back to one, clearing previous fault flags.

        :param auto_calibrate: Whether to calibrate the minimum and maximum positions based on actual motion.
        """
        # clear and then reset ACT
        # rclpy.logging.get_logger('robotiq_urcap_ctrl_ros2').info('Gripper reset')
        self._set_vars({self.ACT: 0})
        # rclpy.logging.get_logger('robotiq_urcap_ctrl_ros2').info('Gripper restarting')
        # rclpy.logging.get_logger('robotiq_urcap_ctrl_ros2').info(f"Griper status {self._get_var(self.STA)}")
        self._set_vars({self.ACT: 1})
        # rclpy.logging.get_logger('robotiq_urcap_ctrl_ros2').info(f"Griper status {self._get_var(self.STA)}")

        # wait for activation to go through
        while not self.is_active():
            time.sleep(0.001)

        # auto-calibrate position range if desired
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self):
        """Returns whether the gripper is active."""
        status = self._get_var(self.STA)
        return status == RobotiqGripper.GripperStatus.ACTIVE.value

    def get_min_position(self):
        """Returns the minimum position the gripper can reach (open position)."""
        return self._min_position

    def get_max_position(self):
        """Returns the maximum position the gripper can reach (closed position)."""
        return self._max_position

    def get_open_position(self):
        """Returns what is considered the open position for gripper (minimum position value)."""
        return self.get_min_position()

    def get_closed_position(self):
        """Returns what is considered the closed position for gripper (maximum position value)."""
        return self.get_max_position()

    def is_open(self):
        """Returns whether the current position is considered as being fully open."""
        return self.get_current_position() <= self.get_open_position()

    def is_closed(self):
        """Returns whether the current position is considered as being fully closed."""
        return self.get_current_position() >= self.get_closed_position()

    def get_current_position(self):
        """Returns the current position as returned by the physical hardware."""
        return self._get_var(self.POS)

    def auto_calibrate(self, log=True):
        """Attempts to calibrate the open and closed positions, by slowly closing and opening the gripper.

        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 1, 1)
        if status != RobotiqGripper.ObjectStatus.AT_DEST.value:
            raise RuntimeError("Calibration failed opening to start: " + str(status))

        # try to close as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.get_closed_position(), 1, 1)
        if status != RobotiqGripper.ObjectStatus.AT_DEST.value:
            raise RuntimeError("Calibration failed because of an object: " + str(status))
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 1, 1)
        if status != RobotiqGripper.ObjectStatus.AT_DEST.value:
            raise RuntimeError("Calibration failed because of an object: " + str(status))
        assert position >= self._min_position
        self._min_position = position

        if log:
            rclpy.logging.get_logger('robotiq_urcap_ctrl_ros2').info(("Gripper auto-calibrated to " +
                  str(self.get_min_position()) +
                  " " +
                  str(self.get_max_position())))

    def move(self, position, speed, force):
        """Sends commands to start moving towards the given position, with the specified speed and force.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = {self.POS: clip_pos,
                    self.SPE: clip_spe,
                    self.FOR: clip_for,
                    self.GTO: 1}
        return self._set_vars(var_dict), clip_pos

    def move_and_wait_for_pos(self, position, speed, force):  # noqa
        """Sends commands to start moving towards the given position, with the specified speed and force, and
        then waits for the move to complete.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with an integer representing the last position returned by the gripper after it notified
        that the move had completed, a status indicating how the move ended (see ObjectStatus enum for details). Note
        that it is possible that the position was not reached, if an object was detected during motion.
        """
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)

        # wait until not moving
        cur_obj = self._get_var(self.OBJ)
        while cur_obj == RobotiqGripper.ObjectStatus.MOVING.value:
            cur_obj = self._get_var(self.OBJ)

        # report the actual position and the object status
        final_pos = self._get_var(self.POS)
        final_obj = cur_obj
        return final_pos, final_obj

    def send_command(self, command):
        self.move(command.r_pr, command.r_sp, command.r_fr)

    def get_status(self):
        message = InputMsg()
        # Assign the values to their respective variables
        message.g_act = self._get_var(self.ACT)
        message.g_gto = self._get_var(self.GTO)
        message.g_sta = self._get_var(self.STA)
        message.g_obj = self._get_var(self.OBJ)
        message.g_flt = self._get_var(self.FLT)
        message.g_pr = self._get_var(self.PRE)
        message.g_po = self._get_var(self.POS)
        # message.gCU  = self._get_var()  # current is not read by this package
        return message


class RobotiqGripperNode(Node):
    def __init__(self, device):
        super().__init__('robotiq2FGripper')
        self.gripper = RobotiqGripper(robot_ip=device)
        self.get_logger().info('Robotiq node has been started')
        self.get_logger().info('Connecting to robot at {}'.format(device))
        self.gripper.connect()
        
        self.get_logger().info('Griper active status: {}'.format(self.gripper.is_active()))
        if not self.gripper.is_active():
            self.get_logger().info('Activating griper')
            self.gripper.activate()
            self.get_logger().info('Griper active status: {}'.format(self.gripper.is_active()))
        
        # The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
        self.pub = self.create_publisher(InputMsg, 'Robotiq2FGripperRobotInput', 10)
        self.pub_gripper = self.create_publisher(Int16, '/robotiq_grip_gap', 10)

        # The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
        self.sub = self.create_subscription(OutputMsg, 'Robotiq2FGripperRobotOutput', self.gripper.send_command, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        status = self.gripper.get_status()
        self.pub.publish(status)
        self.pub_gripper.publish(Int16(data=status.g_po))


def main(args=None):
    rclpy.init(args=args)

    device = sys.argv[1] if len(sys.argv) > 1 else '172.31.1.144'
    node = RobotiqGripperNode(device)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.gripper.disconnect()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()