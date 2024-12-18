import time

import rclpy
from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver_franka import RobotDynamicsClient, RobotDynamicsServer
import dqrobotics as dql
import numpy as np

from dqrobotics.interfaces.vrep import DQ_VrepInterface

# vrep = DQ_VrepInterface()
# vrep.connect("192.168.10.103", 19997, 100, 10)
vrep = None

if __name__ == "__main__":
    rclcpp_init()
    rclpy.init()
    sas_node = rclcpp_Node("dummy_robot_dynamics_client_sas")
    node = rclpy.create_node("dummy_robot_dynamics_client")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin_once(timeout_sec=0.1)
        dynam_provider = RobotDynamicsClient(sas_node, "/franka1")
        rclcpp_spin_some(sas_node)
        while not dynam_provider.is_enabled():
            node.get_logger().info("Waiting for robot dynamics provider to be enabled")
            rclcpp_spin_some(sas_node)
            executor.spin_once(timeout_sec=0.1)
            node.get_logger().info("retrying...")
            time.sleep(0.5)

        node.get_logger().info("Dummy robot dynamics client started")
        while rclpy.ok():
            force = dynam_provider.get_stiffness_force()
            torque = dynam_provider.get_stiffness_torque()
            ee_pose = dynam_provider.get_stiffness_frame_pose()
            if vrep is not None:
                vrep.set_object_pose("xd1", ee_pose)
            node.get_logger().info(f"EE Pose: {ee_pose}")
            node.get_logger().info(f"Force: {force}")
            node.get_logger().info(f"Torque: {torque}")
            rclcpp_spin_some(sas_node)
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rclcpp_shutdown()
        node.destroy_node()
        if vrep is not None:
            vrep.disconnect()
        rclpy.shutdown()
