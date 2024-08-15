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
#include <sas_robot_driver_franka/robot_dynamic/qros_robot_dynamics_server.hpp>

using namespace qros;

RobotDynamicsServer::RobotDynamicsServer(const std::shared_ptr<Node> &node, const std::string& topic_prefix):
    node_(node), topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix),
    child_frame_id_(topic_prefix_ + "_stiffness_frame"), parent_frame_id_(topic_prefix_ + "_base"),
    world_to_base_tf_(0),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
    static_base_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_))
{
    // Strip potential leading slash
    if(child_frame_id_.front() == '/'){child_frame_id_ = child_frame_id_.substr(1);}
    if(parent_frame_id_.front() == '/'){parent_frame_id_ = parent_frame_id_.substr(1);}
    RCLCPP_INFO_STREAM(node_->get_logger(),
        "["+ std::string(node_->get_name()) + "]::Initializing RobotDynamicsServer with prefix " + topic_prefix);
    publisher_cartesian_stiffness_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(topic_prefix + "/get/cartesian_stiffness", 1);
}

geometry_msgs::msg::Transform RobotDynamicsServer::_dq_to_geometry_msgs_transform(const DQ& pose)
{
    geometry_msgs::msg::Transform tf_msg;
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

void RobotDynamicsServer::set_world_to_base_tf(const DQ& world_to_base_tf)
{
    if(world_to_base_tf_==0)
    {
        world_to_base_tf_ = world_to_base_tf;
        _publish_base_static_tf();
    }else
    {
        throw std::runtime_error("["+ std::string(node_->get_name()) + "]::[RobotDynamicsServer]::The world to base transform has already been set");
    }
}

void RobotDynamicsServer::_publish_base_static_tf()
{
    geometry_msgs::msg::TransformStamped base_tf;
    base_tf.set__transform(_dq_to_geometry_msgs_transform(world_to_base_tf_));
    std_msgs::msg::Header header;
    header.set__stamp(node_->now());
    header.set__frame_id(WORLD_FRAME_ID);
    base_tf.set__header(header);
    base_tf.set__child_frame_id(parent_frame_id_);
    static_base_tf_broadcaster_->sendTransform(base_tf);

}


void RobotDynamicsServer::publish_stiffness(const DQ& base_to_stiffness, const Vector3d& force, const Vector3d& torque)
{
    std_msgs::msg::Header header;
    header.set__stamp(node_->now());
    geometry_msgs::msg::WrenchStamped msg;
    msg.set__header(header);
    msg.header.set__frame_id(child_frame_id_);
    msg.wrench.force.set__x(force(0));
    msg.wrench.force.set__y(force(1));
    msg.wrench.force.set__z(force(2));
    msg.wrench.torque.set__x(torque(0));
    msg.wrench.torque.set__y(torque(1));
    msg.wrench.torque.set__z(torque(2));
    publisher_cartesian_stiffness_->publish(msg);
    if(seq_ % REDUCE_TF_PUBLISH_RATE == 0)
    {
        header.set__frame_id(parent_frame_id_);
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.set__transform(_dq_to_geometry_msgs_transform(base_to_stiffness));
        tf_msg.set__header(header);
        tf_msg.set__child_frame_id(child_frame_id_);
        tf_broadcaster_->sendTransform(tf_msg);
    }
}


bool RobotDynamicsServer::is_enabled() const
{
    return true; //Always enabled
}

