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
#include <sas_common/sas_common.h>
#include <sas_conversions/eigen3_std_conversions.h>
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include "hand/qros_effector_driver_franka_hand.h"


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
void get_optional_parameter(const std::string &node_prefix, ros::NodeHandle &nh, const std::string &param_name, T &param)
{
    if(nh.hasParam(node_prefix + param_name))
    {
        sas::get_ros_param(nh,param_name,param);
    }else
    {
        ROS_INFO_STREAM(ros::this_node::getName() + "::Parameter " + param_name + " not found. Using default value. " + std::to_string(param));
    }

}


int main(int argc, char **argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }
    ros::init(argc, argv, "sas_robot_driver_franka_hand_node", ros::init_options::NoSigintHandler);
    ROS_WARN("=====================================================================");
    ROS_WARN("---------------------------Quentin Lin-------------------------------");
    ROS_WARN("=====================================================================");
    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");
    const std::string& effector_driver_provider_prefix = ros::this_node::getName();


    ros::NodeHandle nh;
    qros::EffectorDriverFrankaHandConfiguration robot_driver_franka_hand_configuration;

    sas::get_ros_param(nh,"/robot_ip_address",robot_driver_franka_hand_configuration.robot_ip);

    get_optional_parameter(effector_driver_provider_prefix,nh,"/thread_sampling_time_nsec",robot_driver_franka_hand_configuration.thread_sampeling_time_ns);
    get_optional_parameter(effector_driver_provider_prefix,nh,"/default_force",robot_driver_franka_hand_configuration.default_force);
    get_optional_parameter(effector_driver_provider_prefix,nh,"/default_speed",robot_driver_franka_hand_configuration.default_speed);
    get_optional_parameter(effector_driver_provider_prefix,nh,"/default_epsilon_inner",robot_driver_franka_hand_configuration.default_epsilon_inner);
    get_optional_parameter(effector_driver_provider_prefix,nh,"/default_epsilon_outer",robot_driver_franka_hand_configuration.default_epsilon_outer);

    qros::EffectorDriverFrankaHand franka_hand_driver(
        effector_driver_provider_prefix,
        robot_driver_franka_hand_configuration,
        nh,
        &kill_this_process
    );
    try
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating Franka hand.");
        franka_hand_driver.connect();
        franka_hand_driver.initialize();

        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating control loop.");
        franka_hand_driver.start_control_loop();

    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }catch (...)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::Unknown exception.");
    }
    ROS_INFO_STREAM(ros::this_node::getName()+"::Exiting...");
    franka_hand_driver.deinitialize();
    ROS_INFO_STREAM(ros::this_node::getName()+"::deinitialized.");
    franka_hand_driver.disconnect();
    ROS_INFO_STREAM(ros::this_node::getName()+"::disconnected.");


  return 0;
}
