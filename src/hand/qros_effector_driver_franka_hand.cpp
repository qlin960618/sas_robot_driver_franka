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
#include <sas_robot_driver_franka/interfaces/qros_effector_driver_franka_hand.hpp>

#include <franka/exception.h>

using namespace std::placeholders;
using namespace sas_robot_driver_franka_interfaces;


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
        std::shared_ptr<Node> node,
        std::atomic_bool* break_loops):
        driver_node_prefix_(driver_node_prefix),
        configuration_(configuration),
        node_(node),
        break_loops_(break_loops)
    {
        gripper_sptr_ = nullptr;
        grasp_srv_ = node->create_service<srv::Grasp>(driver_node_prefix_ + "/grasp",
            std::bind(&EffectorDriverFrankaHand::_grasp_srv_callback, this, _1, _2));
        move_srv_ = node->create_service<srv::Move>(driver_node_prefix_ + "/move",
            std::bind(&EffectorDriverFrankaHand::_move_srv_callback, this, _1, _2));
        gripper_status_publisher_ = node->create_publisher<msg::GripperState>(
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


    void EffectorDriverFrankaHand::start_control_loop() {
        sas::Clock clock = sas::Clock(configuration_.thread_sampeling_time_s);
        clock.init();
        RCLCPP_INFO_STREAM(node_->get_logger(),"[EffectorDriverFrankaHand]::start_control_loop::Starting control loop.");
        RCLCPP_WARN_STREAM(node_->get_logger(),"[EffectorDriverFrankaHand]::Gripper READY.");
        while (!(*break_loops_))
        {
            if (!_is_connected()) throw std::runtime_error("[" + std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::start_control_loop::Robot is not connected.");

            if (!status_loop_running_)
            {
                RCLCPP_WARN_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_is_connected::Status loop is not running.");
                break_loops_->store(true);
                break;
            }

            clock.update_and_sleep();
            spin_some(node_);
        }
        RCLCPP_INFO_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::start_control_loop::Exiting control loop.");
    }


    void EffectorDriverFrankaHand::connect()
    {
#ifdef IN_TESTING
        RCLCPP_WARN_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::connect::In testing mode. to DUMMY");
        return;
#endif
        gripper_sptr_ = std::make_shared<franka::Gripper>(configuration_.robot_ip);
        if (!_is_connected()) throw std::runtime_error(
            "[" + std::string(node_->get_name())+
            "]::[EffectorDriverFrankaHand]::connect::Could not connect to the robot.");
    }

    void EffectorDriverFrankaHand::disconnect() noexcept
    {
#ifdef IN_TESTING
        RCLCPP_WARN_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::disconnect::In testing mode. from DUMMY");
        return;
#endif
        RCLCPP_WARN_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::disconnecting...");
        gripper_sptr_->~Gripper();
        gripper_sptr_ = nullptr;
    }

    /**
     * @brief robot_driver_franka::gripper_homing
     */
    void EffectorDriverFrankaHand::gripper_homing()
    {
#ifdef IN_TESTING
        RCLCPP_WARN_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::gripper_homing::In testing mode.");
        return;
#endif
        if (!_is_connected()) throw std::runtime_error(
            "[" + std::string(node_->get_name())+ "]::[EffectorDriverFrankaHand]::gripper_homing::Robot is not connected.");
        auto ret = gripper_sptr_->homing();
        if (!ret)
        {
            throw std::runtime_error("[" + std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::gripper_homing::Failed to home the gripper.");
        }
        RCLCPP_INFO_STREAM(node_->get_logger(),"[EffectorDriverFrankaHand]::gripper_homing::Gripper homed.");
    }

    void EffectorDriverFrankaHand::initialize()
    {
        if (!_is_connected()) throw std::runtime_error(
            "[" + std::string(node_->get_name())+ "]::[EffectorDriverFrankaHand]::initialize::Robot is not connected.");
        gripper_homing();
        // start gripper status loop
        status_loop_thread_ = std::thread(&EffectorDriverFrankaHand::_gripper_status_loop, this);
        // check status loop with timeout
        auto time_now = std::chrono::system_clock::now();
        auto time_out = time_now + std::chrono::seconds(5);
        while (!status_loop_running_)
        {
            if (std::chrono::system_clock::now() > time_out){throw std::runtime_error("[" + std::string(node_->get_name()) + "]::[EffectorDriverFrankaHand]::initialize::Could not start status loop.");}
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void EffectorDriverFrankaHand::deinitialize()
    {
#ifdef IN_TESTING
        RCLCPP_WARN_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::deinitialize::In testing mode.");
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


    bool EffectorDriverFrankaHand::_grasp_srv_callback(const std::shared_ptr<srv::Grasp::Request> req, std::shared_ptr<srv::Grasp::Response> res)
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::Grasping...");
        auto force = req->force;
        auto speed = req->speed;
        auto epsilon_inner = req->epsilon_inner;
        auto epsilon_outer = req->epsilon_outer;
        if (force <= 0.0) force = configuration_.default_force;
        if (speed <= 0.0) speed = configuration_.default_speed;
        if (epsilon_inner <= 0.0) epsilon_inner = configuration_.default_epsilon_inner;
        if (epsilon_outer <= 0.0) epsilon_outer = configuration_.default_epsilon_outer;
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::Width: " + std::to_string(req->width));
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::force: " + std::to_string(force) + " speed: " + std::to_string(speed));
        bool ret = false;
        bool function_ret = true;
        gripper_in_use_.lock();
#ifdef IN_TESTING
        ret = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
#else
        try
        {
            ret = gripper_sptr_->grasp(req->width, speed, force, epsilon_inner, epsilon_outer);
        }catch(franka::CommandException& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::CommandException::" + e.what());
            function_ret = false;
        }catch(franka::NetworkException& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_grasp_srv_callback::NetworkException::" + e.what());
            function_ret = false;
        }
#endif
        gripper_in_use_.unlock();
        res->set__success(ret);
        return function_ret;
    }


    bool EffectorDriverFrankaHand::_move_srv_callback(const std::shared_ptr<srv::Move::Request> req, std::shared_ptr<srv::Move::Response> res)
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_move_srv_callback::Moving...");
        auto speed = req->speed;
        if (speed <= 0.0) speed = configuration_.default_speed;
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_move_srv_callback::Speed: " + std::to_string(speed) + " Width: " + std::to_string(req->width));
        bool ret = false;
        bool function_ret = true;
        gripper_in_use_.lock();
#ifdef IN_TESTING
        ret = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
#else
        try
        {
            ret = gripper_sptr_->move(req->width, speed);
        }catch(franka::CommandException& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_move_srv_callback::CommandException::" + e.what());
            function_ret = false;
        }catch(franka::NetworkException& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_move_srv_callback::NetworkException::" + e.what());
            function_ret = false;
        }
#endif
        gripper_in_use_.unlock();
        res->set__success(ret);
        return function_ret;
    }


    void EffectorDriverFrankaHand::_gripper_status_loop()
    {
        sas::Clock clock = sas::Clock(configuration_.thread_sampeling_time_s);
        RCLCPP_INFO_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::Starting status loop.");
        clock.init();
        try
        {
            status_loop_running_ = true;
            while (status_loop_running_)
            {
#ifdef BLOCK_READ_IN_USED
                bool should_unlock = false;
#endif
                if (!_is_connected()) throw std::runtime_error(
                    "[" + std::string(node_->get_name())+
                    "]::[EffectorDriverFrankaHand]::_gripper_status_loop::Robot is not connected.");
                try
                {
#ifdef IN_TESTING
                    RCLCPP_WARN_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::In testing mode.");
                    throw std::runtime_error("[" + std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::In testing mode.");
#endif
#ifdef BLOCK_READ_IN_USED
                    gripper_in_use_.lock();
                    should_unlock = true;
#endif
                    franka::GripperState gripper_state = gripper_sptr_->readOnce();
#ifdef BLOCK_READ_IN_USED
                    gripper_in_use_.unlock();
#endif
                    msg::GripperState msg;
                    msg.set__width(static_cast<float>(gripper_state.width));
                    msg.set__max_width(static_cast<float>(gripper_state.max_width));
                    msg.set__is_grasped(gripper_state.is_grasped);
                    msg.set__temperature(gripper_state.temperature);
                    msg.set__duration_ms(gripper_state.time.toMSec());
                    gripper_status_publisher_->publish(msg);
                }
                catch (...)
                {
#ifdef BLOCK_READ_IN_USED
                    if (should_unlock) gripper_in_use_.unlock();
#endif
                    RCLCPP_INFO_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::Could not read gripper state. Unavailable.");
                    msg::GripperState msg;
                    msg.width = 0.0;
                    gripper_status_publisher_->publish(msg);
                }

                clock.update_and_sleep();
            }
            status_loop_running_ = false;
        }
        catch (std::exception& e)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::Exception::" + e.
                what());
            status_loop_running_ = false;
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+
                "]::[EffectorDriverFrankaHand]::_gripper_status_loop::Exception::Unknown exception.");
            status_loop_running_ = false;
        }
        RCLCPP_INFO_STREAM(node_->get_logger(),"["+ std::string(node_->get_name())+"]::[EffectorDriverFrankaHand]::_gripper_status_loop::Exiting status loop.");
    }
} // qros
