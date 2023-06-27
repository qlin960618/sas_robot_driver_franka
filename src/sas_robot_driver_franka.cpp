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


#include "sas_robot_driver_franka.h"
#include "sas_clock/sas_clock.h"
#include <dqrobotics/utils/DQ_Math.h>

namespace sas
{
    RobotDriverFranka::RobotDriverFranka(const RobotDriverFrankaConfiguration &configuration, std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
   {
        joint_positions_.resize(7);
        joint_velocities_.resize(7);
        joint_forces_.resize(7);
        //end_effector_pose_.resize(7);
        //joint_positions_buffer_.resize(8,0);
        //end_effector_pose_euler_buffer_.resize(7,0);
        //end_effector_pose_homogenous_transformation_buffer_.resize(10,0);
        std::cout<<configuration.ip_address<<std::endl;

        RobotInterfaceFranka::MODE mode = RobotInterfaceFranka::MODE::None;

        std::cout<<configuration.mode<<std::endl;

        if (configuration_.mode == std::string("None"))
        {
            mode = RobotInterfaceFranka::MODE::None;
        }else if (configuration_.mode == std::string("PositionControl"))
        {
            mode = RobotInterfaceFranka::MODE::PositionControl;
        }else if (configuration_.mode == std::string("VelocityControl"))
        {
            mode = RobotInterfaceFranka::MODE::VelocityControl;
        }else
        {
            throw std::runtime_error(std::string("Wrong mode. ") + std::string("You used ") + configuration_.mode
                                     + std::string(". However, you must use None, PositionControl or VelocityControl"));
        }


        robot_driver_interface_sptr_ = std::make_shared<RobotInterfaceFranka>(configuration.ip_address,
                                                                        mode, //None, PositionControl, VelocityControl
                                                                        RobotInterfaceFranka::HAND::ON);
    }

    RobotDriverFranka::~RobotDriverFranka()=default;


    /**
     * @brief 
     * 
     */
    void RobotDriverFranka::connect() 
    {
        robot_driver_interface_sptr_ ->connect();
    }


    /**
     * @brief 
     * 
     */
    void RobotDriverFranka::initialize()
    {
        robot_driver_interface_sptr_->initialize();
    }


    /**
     * @brief 
     * 
     */
    void RobotDriverFranka::deinitialize()
    {
        robot_driver_interface_sptr_->deinitialize();
    }


    /**
     * @brief 
     * 
     */
    void RobotDriverFranka::disconnect()
    {
        robot_driver_interface_sptr_->disconnect();
    }


    /**
     * @brief 
     * 
     * @return VectorXd 
     */
    VectorXd  RobotDriverFranka::get_joint_positions()
    {
        return robot_driver_interface_sptr_->get_joint_positions();
    }


    /**
     * @brief 
     * 
     * @param desired_joint_positions_rad 
     */
    void RobotDriverFranka::set_target_joint_positions(const VectorXd& desired_joint_positions_rad)
    {
        robot_driver_interface_sptr_->set_target_joint_positions(desired_joint_positions_rad);
    }

    /**
     * @brief RobotDriverFranka::get_joint_velocities
     * @return
     */
    VectorXd RobotDriverFranka::get_joint_velocities()
    {
        joint_velocities_ = robot_driver_interface_sptr_->get_joint_velocities();
        return joint_velocities_;
    }


    /**
     * @brief RobotDriverFranka::set_target_joint_velocities
     * @param desired_joint_velocities
     */
    void RobotDriverFranka::set_target_joint_velocities(const VectorXd &desired_joint_velocities)
    {
        desired_joint_velocities_ = desired_joint_velocities;
        robot_driver_interface_sptr_->set_target_joint_velocities(desired_joint_velocities);
    }

    /**
     * @brief RobotDriverFranka::get_joint_forces
     * @return
     */
    VectorXd RobotDriverFranka::get_joint_forces()
    {
        joint_forces_ = robot_driver_interface_sptr_->get_joint_forces();
        return joint_forces_;
    }

}
