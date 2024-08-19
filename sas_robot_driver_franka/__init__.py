"""
"""
from typing import Union

from sas_robot_driver_franka._qros_franka_robot_dynamics_py import RobotDynamicsClient, RobotDynamicsServer

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver_franka_interfaces.srv import Move, Grasp
from sas_robot_driver_franka_interfaces.msg import GripperState
import os
import threading
from queue import Queue
import time
import struct

MOVE_TOPIC_SUFFIX = "move"
GRASP_TOPIC_SUFFIX = "grasp"
STATUS_TOPIC_SUFFIX = "gripper_status"


class FrankaGripperInterface:
    """
    Non blocking interface to control the Franka gripper
    """

    def __init__(self, node: Node, robot_prefix):
        self.robot_prefix = robot_prefix
        self.node = node
        self.move_service = node.create_client(Move, os.path.join(robot_prefix, MOVE_TOPIC_SUFFIX))
        self._moving = False
        self.grasp_service = node.create_client(Grasp, os.path.join(robot_prefix, GRASP_TOPIC_SUFFIX))
        self._grasping = False
        self.status_subscriber = node.create_subscription(GripperState, os.path.join(robot_prefix, STATUS_TOPIC_SUFFIX),
                                                          self._status_callback, 10)

        self.service_future: Union[rclpy.Future, None] = None

        # gripper state
        self.state_width = None
        self.state_max_width = None
        self.state_temperature = None
        self.state_is_grasped = None
        self.spin_handler = self._default_spin_handler

    def _default_spin_handler(self):
        rclpy.spin_once(self.node)

    def _is_service_ready(self, service: Client):
        try:
            # self.node.get_logger().info("Waiting for service: " + service.service_name)
            ret = service.wait_for_service(timeout_sec=0.1)
            return ret
        except Exception as e:
            self.node.get_logger().info("Service error: " + service.service_name + ": " + str(e))
            return False

    def is_enabled(self):
        if self.state_width is None:
            return False
        if not self._is_service_ready(self.move_service):
            return False
        if not self._is_service_ready(self.grasp_service):
            return False
        return True

    def _status_callback(self, msg: GripperState):
        self.state_width = msg.width
        self.state_max_width = msg.max_width
        self.state_temperature = msg.temperature
        self.state_is_grasped = msg.is_grasped

    def move(self, width, speed=0):
        """
        Move the gripper to a specific width
        :param width: width in meters
        :param speed: speed in meters per second
        :return: None
        """
        if self.is_busy():
            raise Exception("Gripper is already moving or grasping, please wait until the previous action is finished")
        self._move(width, speed)

    def grasp(self, width, force=0, speed=0, epsilon_inner=0, epsilon_outer=0):
        """
        Grasp an object with the gripper
        :param width:
        :param force:
        :param speed:
        :param epsilon_inner:
        :param epsilon_outer:
        :return:
        """
        if self.is_busy():
            raise Exception("Gripper is already moving or grasping, please wait until the previous action is finished")
        self._grasp(width, force, speed, epsilon_inner, epsilon_outer)

    def get_max_width(self):
        """ Get the maximum width of the gripper """
        if not self.is_enabled():
            raise Exception("Gripper is not enabled")
        return self.state_max_width

    def get_width(self):
        """ Get the current width of the gripper """
        if not self.is_enabled():
            raise Exception("Gripper is not enabled")
        return self.state_width

    def get_temperature(self):
        """ Get the temperature of the gripper """
        if not self.is_enabled():
            raise Exception("Gripper is not enabled")
        return self.state_temperature

    def is_grasped(self):
        """ Check if an object is grasped """
        if not self.is_enabled():
            raise Exception("Gripper is not enabled")
        return self.state_is_grasped

    def is_moving(self):
        """ Check if the gripper is currently moving """
        return self._moving

    def is_grasping(self):
        """ Check if the gripper is currently grasping """
        return self._grasping

    def is_busy(self):
        """ Check if the gripper is currently moving or grasping """
        return self._moving or self._grasping or self.service_future is not None

    def is_done(self):
        """ Check if the gripper is done moving or grasping """
        if not self.is_busy():
            self.node.get_logger().warn("Gripper is not moving or grasping")
            return False
        else:
            if self.service_future is not None:
                if self.service_future.done():
                    return True
                return False
            else:
                return True

    def get_result(self):
        """
        Get the result of the last action
        :return:
        """
        if self.service_future is not None:
            if self.service_future.done():
                response = self.service_future.result()
                if isinstance(response, Move.Response):
                    self._moving = False
                elif isinstance(response, Grasp.Response):
                    self._grasping = False
                else:
                    raise Exception("Invalid response type")
                self.service_future = None
                return response.success
            else:
                raise Exception("No result available")
        else:
            raise Exception("No result available")

    def wait_until_done(self):
        """
        Wait until the gripper is done moving or grasping
        :return: success
        """
        if not self.is_enabled():
            raise Exception("Gripper is not enabled")
        if not self.is_busy():
            return
        while not self.is_done():
            self.spin_handler()
            time.sleep(0.01)

    def _move(self, width, speed):
        self._moving = True
        # self.node.get_logger().info("Moving gripper to width: " + str(width) + " speed: " + str(speed))
        request = Move.Request()
        request.width = float(width)
        request.speed = float(speed)
        # self.node.get_logger().info("Calling move service")
        self.service_future = self.move_service.call_async(request)

    def _grasp(self, width, force, speed, epsilon_inner, epsilon_outer):
        self._grasping = True
        # self.node.get_logger().info("Grasping object at width: " + str(width) + " force: " + str(force) + " speed: " + str(speed))
        request = Grasp.Request()
        request.width = float(width)
        request.force = float(force)
        request.speed = float(speed)
        request.epsilon_inner = float(epsilon_inner)
        request.epsilon_outer = float(epsilon_outer)
        # self.node.get_logger().info("Calling grasp service")
        self.service_future = self.grasp_service.call_async(request)


    def set_spin_handler(self, spin_handler):
        self.spin_handler = spin_handler
