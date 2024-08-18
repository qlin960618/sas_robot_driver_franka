#pragma once
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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
// #include <sas_common/sas_common.hpp>
#include <sas_conversions/sas_conversions.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros//static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <dqrobotics/DQ.h>

#define REDUCE_TF_PUBLISH_RATE 10
#define WORLD_FRAME_ID "world"
using namespace rclcpp;

namespace qros {

using namespace DQ_robotics;

class RobotDynamicsServer {
private:
    unsigned int seq_ = 0;
    std::shared_ptr<Node> node_;

    std::string topic_prefix_;
    std::string child_frame_id_;
    std::string parent_frame_id_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_cartesian_stiffness_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_base_tf_broadcaster_;

    DQ world_to_base_tf_ = DQ(0);

    static geometry_msgs::msg::Transform _dq_to_geometry_msgs_transform(const DQ& pose);

    void _publish_base_static_tf();

public:
    RobotDynamicsServer() = delete;
    RobotDynamicsServer(const RobotDynamicsServer&) = delete;
// #ifdef BUILD_PYBIND
    // explicit RobotDynamicsServer(const std::string& node_prefix):RobotDynamicsServer(sas::common::get_static_node_handle(),node_prefix){}
// #endif
    explicit RobotDynamicsServer(const std::shared_ptr<Node> &node,  const std::string& topic_prefix="GET_FROM_NODE");

    void publish_stiffness(const DQ& base_to_stiffness, const Vector3d& force, const Vector3d& torque);

    void set_world_to_base_tf(const DQ& world_to_base_tf);

    bool is_enabled() const;
    std::string get_topic_prefix() const {return topic_prefix_;}

};



} // namespace sas