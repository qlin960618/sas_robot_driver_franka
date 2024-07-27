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
#      1. Quentin Lin (qlin1806@g.ecc.u-tokyo.ac.jp)
#         - Adapted from sas_robot_driver_franka
# ################################################################
*/


#include <sstream>

#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
#include <sas_common/sas_common.hpp>
// #include <sas_conversions/eigen3_std_conversions.hpp>
#include <sas_robot_driver_franka/interfaces/qros_effector_driver_franka_hand.h>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}



template<typename T>
void get_optional_parameter(std::shared_ptr<Node> node, const std::string &param_name, T &param)
{
    if(node->has_parameter(param_name))
    {
        sas::get_ros_parameter(node,param_name,param);
    }else
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "["+std::string(node->get_name())+"]:Parameter " + param_name + " not found. Using default value. " + std::to_string(param));
    }

}


int main(int argc, char **argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_franka_hand_node");

    const auto node_name = std::string(node->get_name());
    RCLCPP_WARN(node->get_logger(),"=====================================================================");
    RCLCPP_WARN(node->get_logger(),"---------------------------Quentin Lin-------------------------------");
    RCLCPP_WARN(node->get_logger(),"=====================================================================");
    RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"["+node_name+"]::Loading parameters from parameter server.");

    qros::EffectorDriverFrankaHandConfiguration robot_driver_franka_hand_configuration;
    sas::get_ros_parameter(node,"robot_ip_address",robot_driver_franka_hand_configuration.robot_ip);
    sas::get_ros_parameter(node,"thread_sampling_time_sec",robot_driver_franka_hand_configuration.thread_sampeling_time_s);
    sas::get_ros_parameter(node,"default_force",robot_driver_franka_hand_configuration.default_force);
    sas::get_ros_parameter(node,"default_speed",robot_driver_franka_hand_configuration.default_speed);
    sas::get_ros_parameter(node,"default_epsilon_inner",robot_driver_franka_hand_configuration.default_epsilon_inner);
    sas::get_ros_parameter(node,"default_epsilon_outer",robot_driver_franka_hand_configuration.default_epsilon_outer);

    qros::EffectorDriverFrankaHand franka_hand_driver(
        node_name,
        robot_driver_franka_hand_configuration,
        node,
        &kill_this_process
    );
    try
    {
        RCLCPP_INFO_STREAM(node->get_logger(),"["+node_name+"]::Instantiating Franka hand.");
        franka_hand_driver.connect();
        franka_hand_driver.initialize();
        RCLCPP_INFO_STREAM(node->get_logger(),"["+node_name+"]::Starting control loop.");
        franka_hand_driver.start_control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "["+node_name+"]::Exception::" + e.what());
    }catch (...)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "["+node_name+"]::Exception::Unknown exception.");
    }
    RCLCPP_INFO_STREAM(node->get_logger(),"["+node_name+"]::Exiting...");
    franka_hand_driver.deinitialize();
    RCLCPP_INFO_STREAM(node->get_logger(),"["+node_name+"]::Deinitialized.");
    franka_hand_driver.disconnect();
    RCLCPP_INFO_STREAM(node->get_logger(),"["+node_name+"]::Disconnected.");


  return 0;
}
