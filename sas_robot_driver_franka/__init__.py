"""
"""
from sas_robot_driver_franka._qros_franka_robot_dynamic import RobotDynamicsClient, RobotDynamicsServer

import rclpy
from rclpy.node import Node
# from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver_franka_interfaces.srv import Move, Move_Request, Move_Response, Grasp, Grasp_Request, Grasp_Response
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
                                                          self._status_callback, 5)

        self.result_queue = Queue()

        # gripper state
        self.state_width = None
        self.state_max_width = None
        self.state_temperature = None
        self.state_is_grasped = None
        self.spin_handler = self._default_spin_handler

    def _default_spin_handler(self):
        rclpy.spin_once(self.node)

    def set_spin_handler(self, spin_handler):
        self.spin_handler = spin_handler

    def _is_service_ready(self, service):
        try:
            self.node.get_logger().info("Waiting for service: " + service.service_name)
            service.wait_for_service(timeout_sec=0.1)
            return True
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
        threading.Thread(target=self._move, args=(width, speed)).start()

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
        threading.Thread(target=self._grasp, args=(width, force, speed, epsilon_inner, epsilon_outer)).start()

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
        return self._moving or self._grasping

    def is_done(self):
        """ Check if the gripper is done moving or grasping """
        if not self.is_busy():
            self.node.get_logger().warn("Gripper is not moving or grasping")
            return False
        else:
            if self.result_queue.empty():
                return False
            else:
                return True

    def get_result(self):
        """
        Get the result of the last action
        :return:
        """
        if not self.result_queue.empty():
            response = self.result_queue.get()
            self._moving = False
            self._grasping = False
            return response.success
        else:
            raise Exception("No result available")

    def wait_until_done(self):
        """
        Wait until the gripper is done moving or grasping
        :return: success
        """
        if not self.is_enabled():
            raise Exception("Gripper is not enabled")
        if not self._moving and not self._grasping:
            raise Exception("Gripper is not moving or grasping")
        if not self._moving or self._grasping:
            raise Exception("Gripper is not moving or grasping")
        while self._moving or self._grasping:
            self.spin_handler()
            time.sleep(0.1)
            if not self.result_queue.empty():
                response = self.result_queue.get()
                if isinstance(response, Move_Response):
                    self._moving = False
                elif isinstance(response, Grasp_Response):
                    self._grasping = False
                else:
                    raise Exception("Invalid response type")
                break

        return response.success

    def _move(self, width, speed):
        self._moving = True
        request = Move_Request()
        request.width = width
        request.speed = speed
        response = self.move_service.call(request)
        self.result_queue.put(response)

    def _grasp(self, width, force, speed, epsilon_inner, epsilon_outer):
        self._grasping = True
        request = Grasp_Request()
        request.width = width
        request.force = force
        request.speed = speed
        request.epsilon_inner = epsilon_inner
        request.epsilon_outer = epsilon_outer
        response = self.grasp_service.call(request)
        self.result_queue.put(response)
