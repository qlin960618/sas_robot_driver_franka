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


#include <sas_robot_driver_franka/sas_robot_driver_franka.hpp>
#include <sas_core/sas_clock.hpp>
#include <dqrobotics/utils/DQ_Math.h>


namespace sas
{
    RobotDriverFranka::RobotDriverFranka(
        const std::shared_ptr<Node> &node,
        const std::shared_ptr<qros::RobotDynamicsServer> &robot_dynamic_provider, const RobotDriverFrankaConfiguration &configuration, std::atomic_bool *break_loops
    ):
    RobotDriver(break_loops),
    node_(node),
    configuration_(configuration),
    robot_dynamic_provider_sptr_(robot_dynamic_provider),
    break_loops_(break_loops)
   {
        joint_positions_.resize(7);
        joint_velocities_.resize(7);
        joint_torques_.resize(7);
        //end_effector_pose_.resize(7);
        //joint_positions_buffer_.resize(8,0);
        //end_effector_pose_euler_buffer_.resize(7,0);
        //end_effector_pose_homogenous_transformation_buffer_.resize(10,0);
        // std::cout<<configuration.ip_address<<std::endl;

        RobotInterfaceFranka::MODE mode = RobotInterfaceFranka::MODE::None;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Mode: " << configuration_.mode);

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


        robot_driver_interface_sptr_ = std::make_shared<RobotInterfaceFranka>(
            configuration.interface_configuration,
            configuration.ip_address,
            mode, //None, PositionControl, VelocityControl
            RobotInterfaceFranka::HAND::OFF
        );
    }

    RobotDriverFranka::~RobotDriverFranka()=default;


    void RobotDriverFranka::_update_stiffness_contact_and_pose() const
    {
        Vector3d force, torque;
        std::tie(force, torque) = robot_driver_interface_sptr_->get_stiffness_force_torque();
        const auto pose = robot_driver_interface_sptr_->get_stiffness_pose();
        robot_dynamic_provider_sptr_->publish_stiffness(pose, force, torque);
    }



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
        if(robot_driver_interface_sptr_->get_err_state()) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                "["+std::string(node_->get_name())+"]::driver interface "
                "error on:"+robot_driver_interface_sptr_->get_status_message());
            break_loops_->store(true);
        }
        if(!ok()) {
            RCLCPP_WARN_STREAM(node_->get_logger(),
                "["+std::string(node_->get_name())+"]::driver interface exit on shotdown signal recieved."
                );
        }
        _update_stiffness_contact_and_pose();
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
        if(robot_driver_interface_sptr_->get_err_state()) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                "["+std::string(node_->get_name())+"]::driver interface "
                "error on:"+robot_driver_interface_sptr_->get_status_message());
            break_loops_->store(true);
        }
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
        // desired_joint_velocities_ = desired_joint_velocities;
        robot_driver_interface_sptr_->set_target_joint_velocities(desired_joint_velocities);
        if(robot_driver_interface_sptr_->get_err_state()) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "["+std::string(node_->get_name())+"]::driver interface "
                                "error on:"+robot_driver_interface_sptr_->get_status_message());
            break_loops_->store(true);
        }
    }

    /**
     * @brief RobotDriverFranka::get_joint_forces
     * @return
     */
    VectorXd RobotDriverFranka::get_joint_torques()
    {
        joint_torques_ = robot_driver_interface_sptr_->get_joint_forces();
        return joint_torques_;
    }

}
