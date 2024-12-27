import time

import rclpy
from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver_franka import RobotDynamicsClient, RobotDynamicsServer
import dqrobotics as dql
import numpy as np




if __name__ == "__main__":
    rclcpp_init()
    rclpy.init()
    sas_node = rclcpp_Node("dummy_robot_dynamics_server_sas")
    node = rclpy.create_node("dummy_robot_dynamics_server")
    dynam_provider = RobotDynamicsServer(sas_node, "/franka1")

    t = dql.DQ([0, 0, 1])
    r = dql.DQ([1, 0, 0, 0])
    base_pose = r + 0.5 * dql.E_ * t * r
    dynam_provider.set_world_to_base_tf(base_pose)
    t_ = 0
    node.get_logger().info("Dummy robot dynamics server started")
    r = dql.DQ([0, 0, 0, 1])
    rate = node.create_rate(100)
    dummy_force = np.random.rand(3) * 100
    dummy_torque = np.random.rand(3) * 10
    try:
        while rclpy.ok():
            t = dql.DQ([1, 2, 1]) + dql.i_ * np.sin(t_/2/np.pi) + dql.j_ * np.cos(t_/2/np.pi)
            ee_pose = r + 0.5 * dql.E_ * t * r
            dummy_force = dummy_force * 0.9 + np.random.rand(3) * 10
            dummy_torque = dummy_torque * 0.9 + np.random.rand(3) * 1
            dynam_provider.publish_stiffness(ee_pose, dummy_force, dummy_torque)
            rclcpp_spin_some(sas_node)
            rclpy.spin_once(node)
            t_ += 0.01
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rclcpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()

