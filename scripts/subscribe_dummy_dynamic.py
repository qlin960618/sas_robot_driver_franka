import rospy
from sas_robot_driver_franka import RobotDynamicsInterface
import dqrobotics as dql
import numpy as np

from dqrobotics.interfaces.vrep import DQ_VrepInterface

vrep = DQ_VrepInterface()
vrep.connect("192.168.10.103", 19997, 100, 10)

if __name__ == "__main__":
    rospy.init_node("dummy_robot_dynamics_subscriber")

    dynam_provider = RobotDynamicsInterface("/franka1")
    while not dynam_provider.is_enabled():
        rospy.loginfo("Waiting for robot dynamics provider to be enabled")
        rospy.sleep(1)

    rospy.loginfo("Subscribing to dummy robot dynamics")
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        force = dynam_provider.get_stiffness_force()
        torque = dynam_provider.get_stiffness_torque()
        ee_pose = dynam_provider.get_stiffness_frame_pose()
        vrep.set_object_pose("xd1", ee_pose)
        rospy.loginfo("EE Pose: %s", ee_pose)
        rospy.loginfo("Force: %s", force)
        rospy.loginfo("Torque: %s", torque)
        rate.sleep()