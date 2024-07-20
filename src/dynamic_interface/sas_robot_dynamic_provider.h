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
#   Author: Juan Jose Quiroz Omana, email: juanjqogm@gmail.com
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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <dqrobotics/DQ.h>

#define REDUCE_TF_PUBLISH_RATE 10

namespace qros {

using namespace DQ_robotics;

class RobotDynamicProvider {
private:
    unsigned int seq_ = 0;

    std::string node_prefix_;
    std::string child_frame_id_;
    std::string parent_frame_id_;

    ros::Publisher publisher_cartesian_stiffness_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;


    static geometry_msgs::Transform _dq_to_geometry_msgs_transform(const DQ& pose) ;

public:
    RobotDynamicProvider(ros::NodeHandle& nodehandle, const std::string& node_prefix=ros::this_node::getName());
    RobotDynamicProvider(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle, const std::string& node_prefix=ros::this_node::getName());

    void publish_stiffness(const DQ& base_to_stiffness, const Vector3d& force, const Vector3d& torque);

};



} // namespace sas