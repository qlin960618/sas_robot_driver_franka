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
#include <robot_dynamic/qros_robot_dynamics_provider.h>

using namespace qros;

RobotDynamicProvider::RobotDynamicProvider(ros::NodeHandle &nodehandle, const std::string &node_prefix):
    RobotDynamicProvider(nodehandle, nodehandle, node_prefix)
{
    //Delegated
}

RobotDynamicProvider::RobotDynamicProvider(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string &node_prefix):
    node_prefix_(node_prefix),
    child_frame_id_(node_prefix + "_stiffness_frame"),
    parent_frame_id_(node_prefix + "_base")
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotDynamicProvider with prefix " + node_prefix);
    publisher_cartesian_stiffness_ = publisher_nodehandle.advertise<geometry_msgs::WrenchStamped>(node_prefix + "/get/cartesian_stiffness", 1);

}


geometry_msgs::Transform RobotDynamicProvider::_dq_to_geometry_msgs_transform(const DQ& pose)
{
    geometry_msgs::Transform tf_msg;
    const auto t = translation(pose);
    const auto r = rotation(pose);
    tf_msg.translation.x = t.q(1);
    tf_msg.translation.y = t.q(2);
    tf_msg.translation.z = t.q(3);
    tf_msg.rotation.w = r.q(0);
    tf_msg.rotation.x = r.q(1);
    tf_msg.rotation.y = r.q(2);
    tf_msg.rotation.z = r.q(3);
    return tf_msg;
}



void RobotDynamicProvider::publish_stiffness(const DQ& base_to_stiffness, const Vector3d& force, const Vector3d& torque)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.seq = seq_++;
    geometry_msgs::WrenchStamped msg;
    msg.header = header;
    msg.header.frame_id = child_frame_id_;
    msg.wrench.force.x = force(0);
    msg.wrench.force.y = force(1);
    msg.wrench.force.z = force(2);
    msg.wrench.torque.x = torque(0);
    msg.wrench.torque.y = torque(1);
    msg.wrench.torque.z = torque(2);
    publisher_cartesian_stiffness_.publish(msg);
    if(seq_ % REDUCE_TF_PUBLISH_RATE == 0)
    {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.transform = _dq_to_geometry_msgs_transform(base_to_stiffness);
        tf_msg.header = header;
        tf_msg.header.frame_id = parent_frame_id_;
        tf_msg.child_frame_id = child_frame_id_;
        tf_broadcaster_.sendTransform(tf_msg);
    }
}


bool RobotDynamicProvider::is_enabled() const
{
    return true; //Always enabled
}

