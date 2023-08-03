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
#include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <thread>
#include <sas_robot_driver/sas_robot_driver_interface.h>

using namespace DQ_robotics;
using namespace Eigen;

namespace sas
{



struct RobotDriverCoppeliaConfiguration
{

    int thread_sampling_time_nsec;
    int port;
    std::string ip;
    std::vector<std::string> jointnames;
    std::string robot_mode;
    bool mirror_mode;
    std::string real_robot_topic_prefix;
};

class RobotDriverCoppelia: public RobotDriver
{
private:
    RobotDriverCoppeliaConfiguration configuration_;

    std::string robot_mode_ = std::string("VelocityControl");   // PositionControl
    bool mirror_mode_ = false;
    double gain_ = 0.5;
    std::string real_robot_topic_prefix_;

    VectorXd current_joint_positions_;
    VectorXd current_joint_velocities_;
    VectorXd current_joint_forces_;


    VectorXd desired_joint_velocities_;
    VectorXd desired_joint_positions_;

    std::atomic<bool> finish_motion_;

    int dim_configuration_space_;

    void _update_robot_state(const VectorXd& q, const VectorXd& q_dot, const VectorXd& forces);

    void finish_motion();

    void _start_joint_velocity_control_mode();
    std::thread joint_velocity_control_mode_thread_;
    void _start_joint_velocity_control_thread();


    void _start_joint_velocity_control_mirror_mode();
    std::thread joint_velocity_control_mirror_mode_thread_;
    void _start_joint_velocity_control_mirror_thread();

    std::shared_ptr<sas::RobotDriverInterface> franka1_ros_;


protected:
    std::shared_ptr<DQ_VrepInterface> vi_;
    std::vector<std::string> jointnames_;



public:
    RobotDriverCoppelia(const RobotDriverCoppelia&)=delete;
    RobotDriverCoppelia()=delete;
    ~RobotDriverCoppelia();

    RobotDriverCoppelia(const RobotDriverCoppeliaConfiguration& configuration, std::atomic_bool* break_loops);


    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    VectorXd get_joint_velocities() override;
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities) override;

    VectorXd get_joint_forces() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;


};
}
