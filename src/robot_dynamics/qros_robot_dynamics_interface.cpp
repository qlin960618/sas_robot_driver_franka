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
#include <robot_dynamic/qros_robot_dynamics_interface.h>

using namespace qros;

RobotDynamicInterface::RobotDynamicInterface(ros::NodeHandle &nodehandle, const std::string &node_prefix):
    RobotDynamicInterface(nodehandle, nodehandle, node_prefix)
{
    //Delegated
}

RobotDynamicInterface::RobotDynamicInterface(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string &node_prefix):
    node_prefix_(node_prefix),
    child_frame_id_(node_prefix + "_stiffness_frame"),
    parent_frame_id_(node_prefix + "_base"),
    tf_buffer_(ros::Duration(BUFFER_DURATION_DEFAULT_S)),
    tf_listener_(tf_buffer_, subscriber_nodehandle),
    last_stiffness_force_(Vector3d::Zero()),
    last_stiffness_torque_(Vector3d::Zero()),
    last_stiffness_frame_pose_(0)
{
    // Strip potential leading slash
    if(child_frame_id_.front() == '/'){child_frame_id_ = child_frame_id_.substr(1);}
    if(parent_frame_id_.front() == '/'){parent_frame_id_ = parent_frame_id_.substr(1);}
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotDynamicInterface with prefix " + node_prefix);
    subscriber_cartesian_stiffness_ = subscriber_nodehandle.subscribe(node_prefix_ + "/get/cartesian_stiffness", 1, &RobotDynamicInterface::_callback_cartesian_stiffness, this);

}

void RobotDynamicInterface::_callback_cartesian_stiffness(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    last_stiffness_force_(0) = msg->wrench.force.x;
    last_stiffness_force_(1) = msg->wrench.force.y;
    last_stiffness_force_(2) = msg->wrench.force.z;

    last_stiffness_torque_(0) = msg->wrench.torque.x;
    last_stiffness_torque_(1) = msg->wrench.torque.y;
    last_stiffness_torque_(2) = msg->wrench.torque.z;

    try {
        const auto transform_stamped = tf_buffer_.lookupTransform( parent_frame_id_, child_frame_id_, ros::Time(0));
        last_stiffness_frame_pose_ = _geometry_msgs_transform_to_dq(transform_stamped.transform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM(ros::this_node::getName() + "::" + ex.what());
    }
}

DQ RobotDynamicInterface::_geometry_msgs_transform_to_dq(const geometry_msgs::Transform& transform)
{
    const auto t = DQ(0,transform.translation.x,transform.translation.y,transform.translation.z);
    const auto r = DQ(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    return r + 0.5 * E_ * t * r;
}

VectorXd RobotDynamicInterface::get_stiffness_force()
{
    if(!is_enabled()) throw std::runtime_error("[RobotDynamicInterface]::calling get_stiffness_force on uninitialized topic");
    return last_stiffness_force_;
}
VectorXd RobotDynamicInterface::get_stiffness_torque()
{
    if(!is_enabled()) throw std::runtime_error("[RobotDynamicInterface]::calling get_stiffness_torque on uninitialized topic");
    return last_stiffness_torque_;
}
DQ RobotDynamicInterface::get_stiffness_frame_pose()
{
    if(!is_enabled()) throw std::runtime_error("[RobotDynamicInterface]::calling get_stiffness_frame_pose on uninitialized Interface");
    return last_stiffness_frame_pose_;
}

bool RobotDynamicInterface::is_enabled() const
{
    if(last_stiffness_frame_pose_==0) return false;
    return true;
}