import rospy
from sas_robot_driver_franka import RobotDynamicsProvider
import dqrobotics as dql
import numpy as np


if __name__ == "__main__":
    rospy.init_node("dummy_robot_dynamics_provider")

    dynam_provider = RobotDynamicsProvider("/franka1")
    t = dql.DQ([0, 0, 1])
    r = dql.DQ([1, 0, 0, 0])
    base_pose = r + 0.5 * dql.E_ * t * r
    dynam_provider.set_world_to_base_tf(base_pose)
    t_ = 0
    rospy.loginfo("Publishing dummy robot dynamics")
    r = dql.DQ([0, 0, 0, 1])
    rate = rospy.Rate(100)
    dummy_force = np.random.rand(3) * 100
    dummy_torque = np.random.rand(3) * 10

    while not rospy.is_shutdown():
        t = dql.DQ([1, 2, 1]) + dql.i_ * np.sin(t_/2/np.pi) + dql.j_ * np.cos(t_/2/np.pi)
        ee_pose = r + 0.5 * dql.E_ * t * r
        dummy_force = dummy_force * 0.9 + np.random.rand(3) * 10
        dummy_torque = dummy_torque * 0.9 + np.random.rand(3) * 1
        dynam_provider.publish_stiffness(ee_pose, dummy_force, dummy_torque)
        rate.sleep()
        t_ += 0.01
