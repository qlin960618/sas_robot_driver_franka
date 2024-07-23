import rospy

from sas_robot_driver_franka import FrankaGripperInterface


def main_loop():
    rate = rospy.Rate(10)  # 10 Hz
    iteration_ = 0

    hand1 = FrankaGripperInterface("/franka_hand_1")

    while not hand1.is_enabled():
        rospy.loginfo("Waiting for gripper to be enabled...")
        rate.sleep()
    rospy.loginfo("Gripper enabled!")

    rate = rospy.Rate(2)  # 0.5 Hz

    while not rospy.is_shutdown():
        rospy.loginfo("Main loop running...")

        # Get the temperature of the gripper
        temperature = hand1.get_temperature()
        rospy.loginfo(f"Temperature: {temperature}")
        max_width = hand1.get_max_width()
        rospy.loginfo(f"Max width: {max_width}")
        width = hand1.get_width()
        rospy.loginfo(f"Width: {width}")
        is_grasped = hand1.is_grasped()
        rospy.loginfo(f"Is grasped: {is_grasped}")
        is_moving = hand1.is_moving()
        rospy.loginfo(f"Is moving: {is_moving}")
        is_grasping = hand1.is_grasping()
        rospy.loginfo(f"Is grasping: {is_grasping}")
        rospy.logwarn("calling move(0.01)")
        if not hand1.is_busy():
            hand1.grasp(0.01)
        else:
            rospy.logwarn("Gripper is busy. Waiting...")
        result_ready = hand1.is_done()
        if not result_ready:
            rospy.loginfo("Waiting for gripper to finish moving...")
        else:
            result = hand1.get_result()
            rospy.loginfo(f"Result: {result}")


        # Check if there is a response in the queue

        iteration_ += 1
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("example_gripper_control_node")
    main_loop()
