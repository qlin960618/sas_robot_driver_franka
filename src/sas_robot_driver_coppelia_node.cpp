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
#         - Adapted from sas_robot_driver_denso_node.cpp
#           (https://github.com/SmartArmStack/sas_robot_driver_denso/blob/master/src/sas_robot_driver_denso_node.cpp)
#      2. Quentin Lin (qlin1806@g.ecc.u-tokyo.ac.jp)
          -Adapted for simplify operation
#
# ################################################################
*/


#include <sstream>

#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
//#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/eigen3_std_conversions.h>
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include "../../src/coppelia/sas_robot_driver_coppelia.h"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}



int main(int argc, char **argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "sas_robot_driver_coppelia_node", ros::init_options::NoSigintHandler);
    ROS_WARN("=====================================================================");
    ROS_WARN("--------------------- Quentin Lin -----------------------------------");
    ROS_WARN("=====================================================================");
    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");


    ros::NodeHandle nh;
    sas::RobotDriverCoppeliaConfiguration robot_driver_coppelia_configuration;
    sas::get_ros_param(nh,"/thread_sampling_time_nsec",robot_driver_coppelia_configuration.thread_sampling_time_nsec);
    sas::get_ros_param(nh,"/vrep_ip",robot_driver_coppelia_configuration.vrep_ip);
    sas::get_ros_param(nh,"/vrep_port",robot_driver_coppelia_configuration.vrep_port);
    sas::get_ros_param(nh,"/vrep_joint_names", robot_driver_coppelia_configuration.vrep_joint_names);
    sas::get_ros_param(nh,"/vrep_dynamically_enabled", robot_driver_coppelia_configuration.vrep_dynamically_enabled);
    sas::get_ros_param(nh,"/robot_mode", robot_driver_coppelia_configuration.robot_mode);
    sas::get_ros_param(nh,"/using_real_robot", robot_driver_coppelia_configuration.using_real_robot);
    sas::get_ros_param(nh,"/robot_topic_prefix", robot_driver_coppelia_configuration.robot_topic_prefix);
    sas::get_ros_param(nh,"/robot_parameter_file_path", robot_driver_coppelia_configuration.robot_parameter_file_path);

    // std::vector<double> q_min_vec, q_max_vec;
    // sas::get_ros_param(nh,"/q_min", q_min_vec);
    // robot_driver_coppelia_configuration.q_min = sas::std_vector_double_to_vectorxd(q_min_vec);
    // sas::get_ros_param(nh,"/q_max", q_max_vec);
    // robot_driver_coppelia_configuration.q_max = sas::std_vector_double_to_vectorxd(q_max_vec);


    try
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating Coppelia robot.");
        sas::RobotDriverCoppelia robot_driver_coppelia(nh, robot_driver_coppelia_configuration,
                                                       &kill_this_process);

        return robot_driver_coppelia.start_control_loop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }


    return 0;
}
