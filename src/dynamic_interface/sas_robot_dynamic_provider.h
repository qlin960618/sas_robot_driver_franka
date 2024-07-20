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
#include <geometry_msgs/PoseStamped.h>
#include <dqrobotics/DQ.h>

namespace sas {

using namespace DQ_robotics;

class RobotDynamicProvider {
private:
    std::string node_prefix_;

    ros::Publisher publisher_cartesian_stiffness_;
    ros::Publisher publisher_stiffness_pose_;

public:
    RobotDynamicProvider(ros::NodeHandle& nodehandle, const std::string& node_prefix=ros::this_node::getName());
    RobotDynamicProvider(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle, const std::string& node_prefix=ros::this_node::getName());

    void publish_stiffness(const DQ& base_to_stiffness, const Vector3d& force, const Vector3d& torque) const;

};



} // namespace sas