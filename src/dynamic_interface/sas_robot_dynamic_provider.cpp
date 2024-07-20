/*
# Copyright (c) 2023 Juan Jose Quiroz Omana
#
#    This file is part of sas_robot_driver_franka.
#
#    sas_robot_driver_franka is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_franka is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjqogm@gmail.com
#
# ################################################################
#
# Contributors:
#      1. Quenitin Lin
#
# ################################################################
*/
#include "sas_robot_dynamic_provider.h"

using namespace sas;

RobotDynamicProvider::RobotDynamicProvider(ros::NodeHandle &nodehandle, const std::string &node_prefix):
    RobotDynamicProvider(nodehandle, nodehandle, node_prefix)
{
    //Delegated
}

RobotDynamicProvider::RobotDynamicProvider(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string &node_prefix):
    node_prefix_(node_prefix)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotDynamicProvider with prefix " + node_prefix);
    publisher_cartesian_stiffness_ = publisher_nodehandle.advertise<geometry_msgs::WrenchStamped>(node_prefix + "/get/cartesian_stiffness", 1);
    publisher_stiffness_pose_ = publisher_nodehandle.advertise<geometry_msgs::PoseStamped>(node_prefix + "/get/stiffness_pose", 1);

}


void RobotDynamicProvider::publish_stiffness(const DQ& base_to_stiffness, const Vector3d& force, const Vector3d& torque) const
{
    publisher_stiffness_pose_.publish(dq_to_geometry_msgs_pose_stamped(base_to_stiffness));
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x = force(0);
    msg.wrench.force.y = force(1);
    msg.wrench.force.z = force(2);
    msg.wrench.torque.x = torque(0);
    msg.wrench.torque.y = torque(1);
    msg.wrench.torque.z = torque(2);
    publisher_cartesian_stiffness_.publish(msg);
}

