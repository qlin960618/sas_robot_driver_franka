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
#      1. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
#         - Adapted from sas_robot_driver_denso.cpp
#           (https://github.com/SmartArmStack/sas_robot_driver_denso/blob/master/src/sas_robot_driver_denso.cpp)
#
# ################################################################
*/


#pragma once
#include <exception>
#include <tuple>
#include <atomic>
#include <vector>
#include <memory>

#include <dqrobotics/DQ.h>

#include <sas_robot_driver/sas_robot_driver.h>
#include "robot_interface_franka.h"
#include <ros/common.h>
#include "sas_robot_dynamic_provider.h"
#include <thread>

using namespace DQ_robotics;
using namespace Eigen;

struct RobotInterfaceFranka::FrankaInterfaceConfiguration;  // Forward declaration

namespace sas
{

struct RobotDriverFrankaConfiguration
{
    std::string ip_address;
    std::string mode;
    int port;
    double speed;
    RobotInterfaceFranka::FrankaInterfaceConfiguration interface_configuration;
};


class RobotDriverFranka: public RobotDriver
{
private:
    RobotDriverFrankaConfiguration configuration_;

    std::shared_ptr<RobotInterfaceFranka> robot_driver_interface_sptr_ = nullptr;

    qros::RobotDynamicProvider* robot_dynamic_provider_;

    //Joint positions
    VectorXd joint_positions_;
    //VectorXd joint_velocities_;
    //VectorXd end_effector_pose_;
    //std::vector<double> joint_positions_buffer_;
    //std::vector<double> end_effector_pose_euler_buffer_;
    //std::vector<double> end_effector_pose_homogenous_transformation_buffer_;

    std::atomic_bool* break_loops_;

    // hotfix function to update cartesian contact and pose information
    void _update_stiffness_contact_and_pose() const;




public:
    //const static int SLAVE_MODE_JOINT_CONTROL;
    //const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    RobotDriverFranka(const RobotDriverFranka&)=delete;
    RobotDriverFranka()=delete;
    ~RobotDriverFranka();

    RobotDriverFranka(
        qros::RobotDynamicProvider* robot_dynamic_provider,
        const RobotDriverFrankaConfiguration& configuration,
        std::atomic_bool* break_loops
        );


    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    VectorXd get_joint_velocities() override;
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities) override;

    VectorXd get_joint_forces() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    //bool set_end_effector_pose_dq(const DQ& pose);
    //DQ get_end_effector_pose_dq();
    
};
}
