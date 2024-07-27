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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <dqrobotics/DQ.h>

#define BUFFER_DURATION_DEFAULT_S 2.0   // 2 second

using namespace rclcpp;
namespace qros {

using namespace DQ_robotics;

class RobotDynamicsClient {
private:
    std::shared_ptr<Node> node_;

    std::string topic_prefix_;
    std::string child_frame_id_;
    std::string parent_frame_id_;

    Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_cartesian_stiffness_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    static geometry_msgs::msg::Transform _dq_to_geometry_msgs_transform(const DQ& pose) ;

    Vector3d last_stiffness_force_;
    Vector3d last_stiffness_torque_;
    DQ last_stiffness_frame_pose_;

    void _callback_cartesian_stiffness(const geometry_msgs::msg::WrenchStamped &msg);

    static DQ _geometry_msgs_transform_to_dq(const geometry_msgs::msg::Transform& transform);

public:
    RobotDynamicsClient() = delete;
    RobotDynamicsClient(const RobotDynamicsClient&) = delete;
// #ifdef BUILD_PYBIND
//     explicit RobotDynamicsClient(const std::string& node_prefix):RobotDynamicsClient(sas::common::get_static_node_handle(),node_prefix){}
// #endif
    explicit RobotDynamicsClient(const std::shared_ptr<Node> &node,  const std::string& topic_prefix="GET_FROM_NODE");

    VectorXd get_stiffness_force();
    VectorXd get_stiffness_torque();
    DQ get_stiffness_frame_pose();

    bool is_enabled() const;
    std::string get_topic_prefix() const {return topic_prefix_;}

};



} // namespace sas