#pragma once
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
#   Author: Quenitin Lin
#
# ################################################################
#
# Contributors:
#      1. Quenitin Lin
#
# ################################################################
*/
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/sas_conversions.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <dqrobotics/DQ.h>

#define BUFFER_DURATION_DEFAULT_S 2.0   // 2 second

namespace qros {

using namespace DQ_robotics;

class RobotDynamicInterface {
private:
    unsigned int seq_ = 0;

    std::string node_prefix_;
    std::string child_frame_id_;
    std::string parent_frame_id_;

    ros::Subscriber subscriber_cartesian_stiffness_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;


    static geometry_msgs::Transform _dq_to_geometry_msgs_transform(const DQ& pose) ;

    Vector3d last_stiffness_force_;
    Vector3d last_stiffness_torque_;
    DQ last_stiffness_frame_pose_;

    void _callback_cartesian_stiffness(const geometry_msgs::WrenchStampedConstPtr &msg);

    static DQ _geometry_msgs_transform_to_dq(const geometry_msgs::Transform& transform);

public:
    RobotDynamicInterface() = delete;
    RobotDynamicInterface(const RobotDynamicInterface&) = delete;
#ifdef BUILD_PYBIND
    explicit RobotDynamicInterface(const std::string& node_prefix):RobotDynamicInterface(sas::common::get_static_node_handle(),node_prefix){}
#endif
    explicit RobotDynamicInterface(ros::NodeHandle& nodehandle, const std::string& node_prefix=ros::this_node::getName());
    RobotDynamicInterface(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle, const std::string& node_prefix=ros::this_node::getName());

    VectorXd get_stiffness_force();
    VectorXd get_stiffness_torque();
    DQ get_stiffness_frame_pose();

    bool is_enabled() const;
    std::string get_topic_prefix() const {return node_prefix_;}

};



} // namespace sas