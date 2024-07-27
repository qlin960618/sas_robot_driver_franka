import threading

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
import rclpy
from sas_robot_driver_franka import FrankaGripperInterface
import time


def main_loop(node):
    iteration_ = 0
    node_name = node.get_name()
    hand1 = FrankaGripperInterface(node, "/franka_hand_1")
    logger = rclpy.node.get_logger(node_name)

    while not hand1.is_enabled():
        logger.info("Waiting for gripper to be enabled...")
        rclcpp_spin_some(node)
        time.sleep(0.1)
    rclpy.node.get_logger(node_name).info("Gripper enabled!")

    def spin_all(node_):
        while rclpy.ok():
            rclcpp_spin_some(node_)
            rclpy.spin_once(node_, timeout_sec=0.1)
            time.sleep(0.1)
            
    thread = threading.Thread(target=spin_all, args=(node, ), daemon=True)
    thread.start()
    
    rate = node.create_rate(2)

    while rclpy.ok():
        logger.info("Main loop running...")

        # Get the temperature of the gripper
        temperature = hand1.get_temperature()
        logger.info(f"Temperature: {temperature}")
        max_width = hand1.get_max_width()
        logger.info(f"Max width: {max_width}")
        width = hand1.get_width()
        logger.info(f"Width: {width}")
        is_grasped = hand1.is_grasped()
        logger.info(f"Is grasped: {is_grasped}")
        is_moving = hand1.is_moving()
        logger.info(f"Is moving: {is_moving}")
        is_grasping = hand1.is_grasping()
        logger.info(f"Is grasping: {is_grasping}")
        logger.warn("calling move(0.01)")
        if not hand1.is_busy():
            hand1.grasp(0.01)
        else:
            logger.warn("Gripper is busy. Waiting...")
        result_ready = hand1.is_done()
        if not result_ready:
            logger.info("Waiting for gripper to finish moving...")
        else:
            result = hand1.get_result()
            logger.info(f"Result: {result}")


        # Check if there is a response in the queue

        iteration_ += 1
        rate.sleep()

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    rclpy.init()
    node_name_ = "example_gripper_control_node"
    NODE = rclpy.create_node(node_name_)
    main_loop(NODE)
