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
#include "hand/qros_effector_driver_franka_hand.h"

#include <franka/exception.h>


namespace qros
{
    //const static int SLAVE_MODE_JOINT_CONTROL;
    //const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    EffectorDriverFrankaHand::~EffectorDriverFrankaHand()
    {
        if (_is_connected())
        {
            disconnect();
        }
    }

    EffectorDriverFrankaHand::EffectorDriverFrankaHand(
        const std::string& driver_node_prefix,
        const EffectorDriverFrankaHandConfiguration& configuration,
        ros::NodeHandle& node_handel,
        std::atomic_bool* break_loops):
        driver_node_prefix_(driver_node_prefix),
        configuration_(configuration),
        node_handel_(node_handel),
        break_loops_(break_loops)
    {
        gripper_sptr_ = nullptr;
        grasp_server_ = node_handel_.advertiseService(driver_node_prefix_ + "/grasp",
                                                      &EffectorDriverFrankaHand::_grasp_srv_callback, this);
        move_server_ = node_handel_.advertiseService(driver_node_prefix_ + "/move",
                                                     &EffectorDriverFrankaHand::_move_srv_callback, this);
        gripper_status_publisher_ = node_handel_.advertise<sas_robot_driver_franka::GripperState>(
            driver_node_prefix_ + "/gripper_status", 1);
    }

    bool EffectorDriverFrankaHand::_is_connected() const
    {
#ifdef IN_TESTING
        return true;
#endif
        if (gripper_sptr_ == nullptr) return false;
        if (!gripper_sptr_) return false;
        else return true;
    }


    void EffectorDriverFrankaHand::start_control_loop()
    {
        sas::Clock clock = sas::Clock(configuration_.thread_sampeling_time_ns);
        clock.init();
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::start_control_loop::Starting control loop.");
        while (!(*break_loops_))
        {
            if (!_is_connected()) throw std::runtime_error(
                "[" + ros::this_node::getName() +
                "]::[EffectorDriverFrankaHand]::start_control_loop::Robot is not connected.");

            if (!status_loop_running_)
            {
                ROS_WARN_STREAM(
                    "["+ ros::this_node::getName()+
                    "]::[EffectorDriverFrankaHand]::_is_connected::Status loop is not running.");
                break_loops_->store(true);
                break;
            }

            clock.update_and_sleep();
            ros::spinOnce();
        }
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::start_control_loop::Exiting control loop.");
    }


    void EffectorDriverFrankaHand::connect()
    {
#ifdef IN_TESTING
        ROS_WARN_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::connect::In testing mode. to DUMMY");
        return;
#endif
        gripper_sptr_ = std::make_shared<franka::Gripper>(configuration_.robot_ip);
        if (!_is_connected()) throw std::runtime_error(
            "[" + ros::this_node::getName() +
            "]::[EffectorDriverFrankaHand]::connect::Could not connect to the robot.");
    }

    void EffectorDriverFrankaHand::disconnect() noexcept
    {
#ifdef IN_TESTING
        ROS_WARN_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::disconnect::In testing mode. from DUMMY");
        return;
#endif
        ROS_WARN_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::disconnecting...");
        gripper_sptr_->~Gripper();
        gripper_sptr_ = nullptr;
    }

    /**
     * @brief robot_driver_franka::gripper_homing
     */
    void EffectorDriverFrankaHand::gripper_homing()
    {
#ifdef IN_TESTING
        ROS_WARN_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::gripper_homing::In testing mode.");
        return;
#endif
        if (!_is_connected()) throw std::runtime_error(
            "[" + ros::this_node::getName() + "]::[EffectorDriverFrankaHand]::gripper_homing::Robot is not connected.");
        auto ret = gripper_sptr_->homing();
        if (!ret)
        {
            throw std::runtime_error(
                "[" + ros::this_node::getName() +
                "]::[EffectorDriverFrankaHand]::gripper_homing::Failed to home the gripper.");
        }
        ROS_INFO_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::gripper_homing::Gripper homed.");
    }

    void EffectorDriverFrankaHand::initialize()
    {
        if (!_is_connected()) throw std::runtime_error(
            "[" + ros::this_node::getName() + "]::[EffectorDriverFrankaHand]::initialize::Robot is not connected.");
        gripper_homing();
        // start gripper status loop
        status_loop_thread_ = std::thread(&EffectorDriverFrankaHand::_gripper_status_loop, this);
    }

    void EffectorDriverFrankaHand::deinitialize()
    {
#ifdef IN_TESTING
        ROS_WARN_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::deinitialize::In testing mode.");
        return;
#endif
        if (_is_connected())
        {
            franka::GripperState gripper_state = gripper_sptr_->readOnce();
            if (gripper_state.is_grasped)
            {
                gripper_sptr_->stop();
            }
        }
    }


    bool EffectorDriverFrankaHand::_grasp_srv_callback(sas_robot_driver_franka::Grasp::Request& req,
                                                       sas_robot_driver_franka::Grasp::Response& res)
    {
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::Grasping...");
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::Width: " + std::
            to_string(req.width));
        auto force = req.force;
        auto speed = req.speed;
        auto epsilon_inner = req.epsilon_inner;
        auto epsilon_outer = req.epsilon_outer;
        if (force <= 0.0) force = configuration_.default_force;
        if (speed <= 0.0) speed = configuration_.default_speed;
        if (epsilon_inner <= 0.0) epsilon_inner = configuration_.default_epsilon_inner;
        if (epsilon_outer <= 0.0) epsilon_outer = configuration_.default_epsilon_outer;
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::force: " + std::
            to_string(force) + " speed: " + std::to_string(speed));
        bool ret = false;
        bool function_ret = true;
        gripper_in_use_.lock();
#ifdef IN_TESTING
        ret = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
#else
        try
        {
            ret = gripper_sptr_->grasp(req.width, speed, force, epsilon_inner, epsilon_outer);
        }catch(franka::CommandException& e)
        {
            ROS_ERROR_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::CommandException::" + e.what());
            function_ret = false;
        }catch(franka::NetworkException& e)
        {
            ROS_ERROR_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::NetworkException::" + e.what());
            function_ret = false;
        }
#endif
        gripper_in_use_.unlock();
        res.success = ret;
        return function_ret;
    }


    bool EffectorDriverFrankaHand::_move_srv_callback(sas_robot_driver_franka::Move::Request& req,
                                                      sas_robot_driver_franka::Move::Response& res)
    {
        ROS_INFO_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_move_srv_callback::Moving...");
        auto speed = req.speed;
        if (speed <= 0.0) speed = configuration_.default_speed;
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_move_srv_callback::Speed: " + std::to_string
            (speed) + " Width: " + std::to_string(req.width));
        bool ret = false;
        bool function_ret = true;
        gripper_in_use_.lock();
#ifdef IN_TESTING
        ret = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
#else
        try
        {
            ret = gripper_sptr_->move(req.width, speed);
        }catch(franka::CommandException& e)
        {
            ROS_ERROR_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_move_srv_callback::CommandException::" + e.what());
            function_ret = false;
        }catch(franka::NetworkException& e)
        {
            ROS_ERROR_STREAM("["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_move_srv_callback::NetworkException::" + e.what());
            function_ret = false;
        }
#endif
        gripper_in_use_.unlock();
        res.success = ret;
        return function_ret;
    }


    void EffectorDriverFrankaHand::_gripper_status_loop()
    {
        status_loop_running_ = true;
        sas::Clock clock = sas::Clock(configuration_.thread_sampeling_time_ns);
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::Starting status loop.")
        ;
        clock.init();
        try
        {
            while (status_loop_running_)
            {
                bool should_unlock = false;
                if (!_is_connected()) throw std::runtime_error(
                    "[" + ros::this_node::getName() +
                    "]::[EffectorDriverFrankaHand]::_gripper_status_loop::Robot is not connected.");
                try
                {
#ifdef IN_TESTING
                    ROS_WARN_STREAM(
                        "["+ ros::this_node::getName()+
                        "]::[EffectorDriverFrankaHand]::_gripper_status_loop::In testing mode.");
                    throw std::runtime_error(
                        "[" + ros::this_node::getName() +
                        "]::[EffectorDriverFrankaHand]::_gripper_status_loop::In testing mode.");
#endif
#ifdef BLOCK_READ_IN_USED
                    gripper_in_use_.lock();
                    should_unlock = true;
#endif
                    franka::GripperState gripper_state = gripper_sptr_->readOnce();
#ifdef BLOCK_READ_IN_USED
                    gripper_in_use_.unlock();
#endif
                    sas_robot_driver_franka::GripperState msg;
                    msg.width = static_cast<float>(gripper_state.width);
                    msg.max_width = static_cast<float>(gripper_state.max_width);
                    msg.is_grasped = gripper_state.is_grasped;
                    msg.temperature = gripper_state.temperature;
                    msg.duration_ms = gripper_state.time.toMSec();
                    gripper_status_publisher_.publish(msg);
                }
                catch (...)
                {
#ifdef BLOCK_READ_IN_USED
                    if (should_unlock) gripper_in_use_.unlock();
#endif
                    ROS_INFO_STREAM(
                        "["+ ros::this_node::getName()+
                        "]::[EffectorDriverFrankaHand]::_gripper_status_loop::Could not read gripper state. Unavailable.")
                    ;
                    sas_robot_driver_franka::GripperState msg;
                    msg.width = 0.0;
                    gripper_status_publisher_.publish(msg);
                }

                clock.update_and_sleep();
            }
            status_loop_running_ = false;
        }
        catch (std::exception& e)
        {
            ROS_ERROR_STREAM(
                "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::Exception::" + e.
                what());
            status_loop_running_ = false;
        }
        catch (...)
        {
            ROS_ERROR_STREAM(
                "["+ ros::this_node::getName()+
                "]::[EffectorDriverFrankaHand]::_gripper_status_loop::Exception::Unknown exception.");
            status_loop_running_ = false;
        }
        ROS_INFO_STREAM(
            "["+ ros::this_node::getName()+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::Exiting status loop.");
    }
} // qros
