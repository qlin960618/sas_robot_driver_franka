/*
# Copyright (c) 2024 Quenitin Lin
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
#   Author: Quenitin Lin
#
# ################################################################
#
# Contributors:
#      1. Quenitin Lin
#
# ################################################################
*/
#include <memory>
#include <sas_robot_driver_franka/robot_dynamic/qros_robot_dynamics_client.hpp>

using namespace qros;


RobotDynamicsClient::RobotDynamicsClient(const std::shared_ptr<Node> &node, const std::string& topic_prefix):
    node_(node), topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix),
    child_frame_id_(topic_prefix_ + "_stiffness_frame"), parent_frame_id_(topic_prefix_ + "_base"),
    last_stiffness_force_(Vector3d::Zero()),
    last_stiffness_torque_(Vector3d::Zero()),
    last_stiffness_frame_pose_(0)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Strip potential leading slash
    if(child_frame_id_.front() == '/'){child_frame_id_ = child_frame_id_.substr(1);}
    if(parent_frame_id_.front() == '/'){parent_frame_id_ = parent_frame_id_.substr(1);}
    RCLCPP_INFO_STREAM(node_->get_logger(),
        "["+ std::string(node_->get_name()) + "]::Initializing RobotDynamicsClient with prefix " + topic_prefix);
    subscriber_cartesian_stiffness_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(topic_prefix + "/get/cartesian_stiffness", 1,
        std::bind(&RobotDynamicsClient::_callback_cartesian_stiffness, this, std::placeholders::_1)
        );

}

void RobotDynamicsClient::_callback_cartesian_stiffness(const geometry_msgs::msg::WrenchStamped &msg)
{
    last_stiffness_force_(0) = msg.wrench.force.x;
    last_stiffness_force_(1) = msg.wrench.force.y;
    last_stiffness_force_(2) = msg.wrench.force.z;

    last_stiffness_torque_(0) = msg.wrench.torque.x;
    last_stiffness_torque_(1) = msg.wrench.torque.y;
    last_stiffness_torque_(2) = msg.wrench.torque.z;

    try {
        const auto transform_stamped = tf_buffer_->lookupTransform( parent_frame_id_, child_frame_id_, tf2::TimePointZero);
        last_stiffness_frame_pose_ = _geometry_msgs_transform_to_dq(transform_stamped.transform);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "["+ std::string(node_->get_name()) + "]::" + ex.what());
    }
}

DQ RobotDynamicsClient::_geometry_msgs_transform_to_dq(const geometry_msgs::msg::Transform& transform)
{
    const auto t = DQ(0,transform.translation.x,transform.translation.y,transform.translation.z);
    const auto r = DQ(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
    return r + 0.5 * E_ * t * r;
}

VectorXd RobotDynamicsClient::get_stiffness_force()
{
    if(!is_enabled()) throw std::runtime_error("[RobotDynamicsClient]::calling get_stiffness_force on uninitialized topic");
    return last_stiffness_force_;
}
VectorXd RobotDynamicsClient::get_stiffness_torque()
{
    if(!is_enabled()) throw std::runtime_error("[RobotDynamicsClient]::calling get_stiffness_torque on uninitialized topic");
    return last_stiffness_torque_;
}
DQ RobotDynamicsClient::get_stiffness_frame_pose()
{
    if(!is_enabled()) throw std::runtime_error("[RobotDynamicsClient]::calling get_stiffness_frame_pose on uninitialized Interface");
    return last_stiffness_frame_pose_;
}

bool RobotDynamicsClient::is_enabled() const
{
    if(last_stiffness_frame_pose_==0) return false;
    return true;
}