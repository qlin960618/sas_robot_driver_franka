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
#         - Original implementation
#
# ################################################################
*/


#include "robot_interface_franka.h"


/**
 * @brief robot_driver_franka::robot_driver_franka
 * @param ROBOT_IP The IP address of the FCI
 * @param mode The operation mode {None, PositionControl}.
 * @param hand The hand option {ONFinished, OFF}.
 */
RobotInterfaceFranka::RobotInterfaceFranka(const std::string &ROBOT_IP,
                                         const MODE& mode,
                                         const HAND& hand):ip_(ROBOT_IP),mode_(mode)
{
    _set_driver_mode(mode);
    if (hand == RobotInterfaceFranka::HAND::ON)
    {
        gripper_sptr_ = std::make_shared<franka::Gripper>(ip_);
    }
    robot_sptr_ = std::make_shared<franka::Robot>(ip_);
    _setDefaultRobotBehavior();
    /*
    std::cout<<"---------------------------------------------------------------"<<std::endl;
    std::cout<<"The connection is not automatic. Expected Workflow:            "<<std::endl;
    std::cout<<"franka_driver.connect();                                       "<<std::endl;
    std::cout<<"franka_driver.initialize();                                    "<<std::endl;
    std::cout<<"                                                               "<<std::endl;
    std::cout<<"franka_driver.deinitialize();                                  "<<std::endl;
    std::cout<<"franka_driver.disconnect();                                    "<<std::endl;
    std::cout<<"---------------------------------------------------------------"<<std::endl;
    */


    std::cout<<"-----------------------------------------------------------------"<<std::endl;
    std::cout<<"RobotInterfaceFranka is brought to you by Juan Jose Quiroz Omana."<<std::endl;
    std::cout<<"                                                                 "<<std::endl;
    std::cout<<"-----------------------------------------------------------------"<<std::endl;
    finish_motion_ = false;
    finish_echo_robot_state_ = false;
}


/**
 * @brief robot_driver_franka::_set_driver_mode sets the mode.
 * @param mode
 */
void RobotInterfaceFranka::_set_driver_mode(const RobotInterfaceFranka::MODE& mode)
{
    switch (mode)
    {
    case RobotInterfaceFranka::MODE::None:
        mode_ = mode;
        break;
    case RobotInterfaceFranka::MODE::PositionControl:
        mode_ = mode;
        break;
    case RobotInterfaceFranka::MODE::VelocityControl:
        mode_ = mode;
        break;
    case RobotInterfaceFranka::MODE::ClearPositions:
        throw std::runtime_error(std::string("The robot_driver_franka::ClearPositions is not available. "));
        break;
    case RobotInterfaceFranka::MODE::Homing:
        throw std::runtime_error(std::string("The robot_driver_franka::Homing is not available. "));
        break;
    case RobotInterfaceFranka::MODE::ForceControl:
        throw std::runtime_error(std::string("The robot_driver_franka::ForceControl is not available. "));
        break;
    }
}


/**
 * @brief robot_driver_franka::connect starts the connection with the robot and starts the robot state thread.
 */
void RobotInterfaceFranka::connect()
{
    _update_status_message("Connecting...", verbose_);
    current_joint_positions_ = read_once_initial_positions(); //_read_once_smooth_initial_positions(1000);
    desired_joint_positions_ = current_joint_positions_;
    desired_joint_velocities_ = VectorXd::Zero(7);
    _update_status_message("Connected.", verbose_);
    _start_echo_robot_state_mode_thread(); //  start_robot_state_loop to read joints without moving the robot;

}


/**
 * @brief robot_driver_franka::disconnect stops all loops threads. Use this method after to run deinitialize().
 */
void RobotInterfaceFranka::disconnect()
{
    _update_status_message("Disconnecting...", verbose_);

    _finish_echo_robot_state();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if (joint_position_control_mode_thread_.joinable())
    {
        joint_position_control_mode_thread_.join();
    }
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }
    if (joint_velocity_control_mode_thread_.joinable())
    {
        joint_velocity_control_mode_thread_.join();
    }

    _update_status_message("Disconnected.", verbose_);
    _restart_default_parameters();


}

void RobotInterfaceFranka::set_target_joint_velocities(const VectorXd &target_joint_velocities)
{
    desired_joint_velocities_ = target_joint_velocities;
}


/**
 * @brief robot_driver_franka::_restart_default_parameters
 */
void RobotInterfaceFranka::_restart_default_parameters()
{
    finish_motion_           = false;
    finish_echo_robot_state_ = false;
    initialize_flag_         = false;
    time_ = 0;
}


/**
 * @brief robot_driver_franka::_compute_recursive_mean computes the recursive mean of a vector
 * @param n The number of samples
 * @param q The vector
 * @return The recursive mean of n samples.
 */
VectorXd RobotInterfaceFranka::_compute_recursive_mean(const double& n, const VectorXd& q)
{
    //Vector7d qm;
    Vector7d qmean;
    for (int i=0;i<q.size();i++)
    {
        //qm[i] = (1/n)*((n-1)*q[i] + q[i]);
        qmean[i]  = (1/n)*((n-1)*q[i] + q[i]);
    }
    return qmean;
}


/**
 * @brief robot_driver_franka::_start_joint_position_control_thread
 */
void RobotInterfaceFranka::_start_joint_position_control_thread()
{
    finish_motion_ = false;
    _update_status_message("checking joint position control thread",verbose_);
    if (joint_position_control_mode_thread_.joinable())
    {
        joint_position_control_mode_thread_.join();
    }
    _update_status_message("Starting joint position control thread",verbose_);
    joint_position_control_mode_thread_ = std::thread(&RobotInterfaceFranka::_start_joint_position_control_mode, this);
}



void RobotInterfaceFranka::_start_joint_velocity_control_thread()
{
    finish_motion_ = false;
    _update_status_message("checking joint velocity control thread",verbose_);
    if (joint_velocity_control_mode_thread_.joinable())
    {
        joint_velocity_control_mode_thread_.join();
    }
    _update_status_message("Starting joint velocity control thread",verbose_);
    joint_velocity_control_mode_thread_ = std::thread(&RobotInterfaceFranka::_start_joint_velocity_control_mode, this);
}

/**
 * @brief robot_driver_franka::_start_echo_robot_state_mode_thread
 */
void RobotInterfaceFranka::_start_echo_robot_state_mode_thread()
{
    finish_echo_robot_state_ = false;
    _update_status_message("Checking echo robot state thread",verbose_);
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }
    _update_status_message("Starting echo robot state thread",verbose_);
    echo_robot_state_mode_thread_ = std::thread(&RobotInterfaceFranka::_start_echo_robot_state_mode, this);
}


/**
 * @brief robot_driver_franka::finish_motion
 */
void RobotInterfaceFranka::finish_motion()
{
    _update_status_message("Ending control loop.",verbose_);
    for (int i=0;i<1000;i++)
    {
        set_target_joint_positions(current_joint_positions_);
        set_target_joint_velocities(VectorXd::Zero(7));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    finish_motion_ = true;
}


/**
 * @brief robot_driver_franka::get_robot_pointer
 * @return
 */
std::shared_ptr<franka::Robot> RobotInterfaceFranka::get_robot_pointer()
{
    return robot_sptr_;
}


/**
 * @brief robot_driver_franka::_finish_echo_robot_state
 */
void RobotInterfaceFranka::_finish_echo_robot_state()
{
    _update_status_message("Finishing echo robot state.",verbose_);
    finish_echo_robot_state_ = true;     
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


/**
 * @brief robot_driver_franka::deinitialize
 */
void RobotInterfaceFranka::deinitialize()
{
    _update_status_message("Deinitializing....",verbose_);
    initialize_flag_ = false;
    switch (mode_)
    {
    case RobotInterfaceFranka::MODE::None:
        break;
    case RobotInterfaceFranka::MODE::PositionControl:
        finish_motion();
        break;
    case RobotInterfaceFranka::MODE::VelocityControl:
        finish_motion();
        break;
    case RobotInterfaceFranka::MODE::ClearPositions:
        throw std::runtime_error(std::string("The robot_driver_franka::ClearPositions is not available. "));
        break;
    case RobotInterfaceFranka::MODE::Homing:
        throw std::runtime_error(std::string("The robot_driver_franka::Homing is not available. "));
        break;
    case RobotInterfaceFranka::MODE::ForceControl:
        throw std::runtime_error(std::string("The robot_driver_franka::ForceControl is not available. "));
        break;
    }
    _update_status_message("Deinitialized.",verbose_);
}


/**
 * @brief robot_driver_franka::initialize
 */
void RobotInterfaceFranka::initialize()
{
    _update_status_message("Initializing....",verbose_);

    initialize_flag_ = true;


    switch (mode_)
    {
    case RobotInterfaceFranka::MODE::None:
        break;
    case RobotInterfaceFranka::MODE::PositionControl:
        _finish_echo_robot_state();
        _start_joint_position_control_thread();
        break;
    case RobotInterfaceFranka::MODE::VelocityControl:
        _finish_echo_robot_state();
        _start_joint_velocity_control_thread();
        break;
    case RobotInterfaceFranka::MODE::ClearPositions:
        throw std::runtime_error(std::string("The robot_driver_franka::ClearPositions is not available. "));
        break;
    case RobotInterfaceFranka::MODE::Homing:
        throw std::runtime_error(std::string("The robot_driver_franka::Homing is not available. "));
        break;
    case RobotInterfaceFranka::MODE::ForceControl:
        throw std::runtime_error(std::string("The robot_driver_franka::ForceControl is not available. "));
        break;
    }
    _update_status_message("Initialized.",verbose_);
}


/**
 * @brief robot_driver_franka::_update_robot_state
 * @param robot_state
 * @param time
 */
void RobotInterfaceFranka::_update_robot_state(const franka::RobotState &robot_state, const double& time)
{
    current_joint_positions_array_ = robot_state.q_d;
    current_joint_positions_ = Eigen::Map<VectorXd>(current_joint_positions_array_.data(), 7);

    current_joint_velocities_array_ = robot_state.dq_d;
    current_joint_velocities_ = Eigen::Map<VectorXd>(current_joint_velocities_array_.data(), 7);

    current_joint_forces_array_ = robot_state.tau_J;
    current_joint_forces_ =  Eigen::Map<VectorXd>(current_joint_forces_array_.data(), 7);

    robot_mode_ = robot_state.robot_mode;
    time_ = time;
}


/**
 * @brief robot_driver_franka::_start_echo_robot_state_mode
 */
void RobotInterfaceFranka::_start_echo_robot_state_mode(){
    double time = 0.0;
    size_t count = 0;
    double t1 = 0.0;

    try {
        robot_sptr_->read([&t1, &count, &time, this](const franka::RobotState& robot_state) {
            if (count == 0)
             {
                t1 = robot_state.time.toSec();
             }
            time = robot_state.time.toSec() - t1;
            _update_robot_state(robot_state, time);
            count++;
            return !finish_echo_robot_state_;
        });
        _update_status_message("Finished echo_robot_state.",verbose_);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
    }
}


/**
 * @brief robot_driver_franka::_start_joint_position_control_mode
 */
void RobotInterfaceFranka::_start_joint_position_control_mode()
{
    std::array<double, 7> initial_position;
    VectorXd target_position;
    double time = 0.0;

    VectorXd q = VectorXd::Zero(7);
    franka::RobotState state = robot_sptr_->readOnce();
    q = Map<VectorXd>(state.q_d.data(), 7);
    desired_joint_positions_ = q;

    VectorXd q_dot = VectorXd::Zero(7);
    _update_status_message("Starting joint position control mode EXPERIMENTAL",verbose_);

    /*
    trajectory_generator_sptr_ =
        std::make_unique<QuadraticProgramMotionGenerator>(1.0, q, q_dot, q);

    double n2 = 0.8;
    double n1 = 1000*std::sqrt(n2);
    VectorXd K2 = (VectorXd(7)<<n2, n2, n2, n2, n2, n2, n2).finished();
    VectorXd K1 = (VectorXd(7)<<n1, n1, n1, n1, 3*n1, 3*n1, 3*n1).finished();
    trajectory_generator_sptr_->set_diagonal_gains(K1, K2);
    */

    custom_generator_sptr_ = std::make_unique<CustomMotionGeneration>(0.9, q, q_dot, q);
    custom_generator_sptr_->set_proportional_gain(1.0);

    finish_motion_ = false;

        try {
            robot_sptr_->control( //------------------------------------------------------------
                [&initial_position, &time, this](const franka::RobotState& robot_state,
                                                 franka::Duration period) -> franka::JointPositions {
                    time += period.toSec();
                    double T = period.toSec();

                    auto new_q = custom_generator_sptr_->compute_new_configuration(desired_joint_positions_, T);


                    if (time == 0.0) {
                        initial_position = robot_state.q_d;
                        new_q = Eigen::Map<VectorXd>(initial_position.data(), 7);
                    }


                    franka::JointPositions output = {{new_q[0], new_q[1],
                                                      new_q[2], new_q[3],
                                                      new_q[4], new_q[5],
                                                      new_q[6]}};

                    if (time == 0.0) {
                        _update_status_message("joint position control mode running! ",verbose_);
                    }

                    if (finish_motion_) {
                        _update_status_message("Motion finished",verbose_);
                        finish_motion_ = false;
                        return franka::MotionFinished(output);
                    }else
                    {
                        _update_robot_state(robot_state, time);
                    }
                    return output;
                }
                );//------------------------------------------------------------

        }
        catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
        }




}


/**
 * @brief RobotInterfaceFranka::_start_joint_velocity_control_mode
 */
void RobotInterfaceFranka::_start_joint_velocity_control_mode()
{
    finish_motion_ = false;

    std::array<double, 7> initial_velocity;

    VectorXd q_dot_initial = VectorXd::Zero(7);
    VectorXd q_dot = VectorXd::Zero(7);
    desired_joint_velocities_ = VectorXd::Zero(7);

    /*
    trajectory_generator_sptr_ =
        std::make_unique<QuadraticProgramMotionGenerator>(0.8, q_dot_initial, q_dot);

    double n2 = 1;
    double n1 = 10*std::sqrt(n2);
    VectorXd K2 = (VectorXd(7)<<n2, n2, n2, n2, n2, n2, n2).finished();
    VectorXd K1 = (VectorXd(7)<<n1, n1, n1, n1, 2*n1, 2*n1, 2*n1).finished();
    trajectory_generator_sptr_->set_diagonal_gains(K1, K2);
    */

    custom_generator_sptr_ = std::make_unique<CustomMotionGeneration>(0.8, q_dot_initial, q_dot);
    custom_generator_sptr_->set_proportional_gain(15.0);


    _update_status_message("Starting joint velocity control mode EXPERIMENTAL",verbose_);
    try {
        double time = 0.0;

        robot_sptr_->control(
            [&initial_velocity, &time, this](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                time += period.toSec();
                double T = period.toSec();

                //auto new_u = trajectory_generator_sptr_->compute_new_configuration_velocities(desired_joint_velocities_, T);
                auto new_u = custom_generator_sptr_->compute_new_configuration_velocities(desired_joint_velocities_, T);

                franka::JointVelocities velocities = {{new_u[0], new_u[1],
                                                       new_u[2], new_u[3],
                                                       new_u[4], new_u[5],
                                                       new_u[6]}};

                if (time == 0.0) {
                    _update_status_message("joint velocity control mode running! ",verbose_);
                }

                if (finish_motion_) {
                    _update_status_message("Motion finished",verbose_);
                    finish_motion_ = false;
                    return franka::MotionFinished(velocities);
                }else
                {
                    _update_robot_state(robot_state, time);
                    initial_velocity = robot_state.dq_d;
                }
                return velocities;
            });
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
    }
}



/**
 * @brief robot_driver_franka::_setDefaultRobotBehavior
 */
void RobotInterfaceFranka::_setDefaultRobotBehavior()
{
    robot_sptr_->setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    robot_sptr_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_sptr_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}


/**
 * @brief robot_driver_franka::_update_status_message
 * @param message
 * @param verbose
 */
void RobotInterfaceFranka::_update_status_message(const std::string &message, const bool& verbose)
{
    status_message_ = message;
    std::string base = "RobotInterfaceFranka::";
    if(verbose)
    {
        std::cout<<base+message<<std::endl;
    }
}


/**
 * @brief robot_driver_franka::get_status_message
 * @return
 */
std::string RobotInterfaceFranka::get_status_message()
{
    return status_message_;
}


/**
 * @brief RobotInterfaceFranka::get_robot_mode
 * @return
 */
std::string RobotInterfaceFranka::get_robot_mode()
{
    std::string string_robot_mode;
    switch (robot_mode_) {
    case (franka::RobotMode::kUserStopped):
        string_robot_mode = "User stopped";
        break;
    case (franka::RobotMode::kIdle):
        string_robot_mode = "Idle";
        break;
    case (franka::RobotMode::kMove):
        string_robot_mode = "Move";
        break;
    case (franka::RobotMode::kGuiding):
        string_robot_mode = "Guiding";
        break;
    case (franka::RobotMode::kReflex):
        string_robot_mode = "Reflex";
        break;
    case (franka::RobotMode::kAutomaticErrorRecovery):
        string_robot_mode = "Automatic error recovery";
        break;
    case (franka::RobotMode::kOther):
         string_robot_mode = "Other";
        break;

    }
    return  string_robot_mode;
}


/**
 * @brief robot_driver_franka::_check_if_robot_is_connected
 */
void RobotInterfaceFranka::_check_if_robot_is_connected()
{
    if(!robot_sptr_)
        throw std::runtime_error("Invalid robot pointer. You must connect(), then initialize(). "
                   + std::string("Example:  robot_driver_franka(IP, robot_driver_franka::MODE::None, robot_driver_franka::HAND::ON)" ));
}


/**
  * @brief robot_driver_franka::_check_if_hand_is_connected
  */
void RobotInterfaceFranka::_check_if_hand_is_connected()
{
    if(!gripper_sptr_)
        throw std::runtime_error("Invalid hand pointer. You must connect(), then initialize(). "
                   + std::string("Example:  robot_driver_franka(IP, robot_driver_franka::MODE::None, robot_driver_franka::HAND::ON)" ));
}


/**
 * @brief robot_driver_franka::get_joint_positions
 * @return
 */
VectorXd RobotInterfaceFranka::get_joint_positions()
{
    _check_if_robot_is_connected();
    return current_joint_positions_;
}


/**
 * @brief robot_driver_franka::get_joint_velocities
 * @return
 */
VectorXd RobotInterfaceFranka::get_joint_velocities()
{
    _check_if_robot_is_connected();
    return current_joint_velocities_;
}


/**
 * @brief RobotInterfaceFranka::get_joint_forces
 * @return
 */
VectorXd RobotInterfaceFranka::get_joint_forces()
{
    _check_if_robot_is_connected();
    return current_joint_forces_;
}


/**
 * @brief robot_driver_franka::get_time
 * @return
 */
double RobotInterfaceFranka::get_time()
{
    _check_if_robot_is_connected();
    return time_;
}


/**
 * @brief robot_driver_franka::_read_once_smooth_initial_positions
 * @param samples
 * @return
 */
VectorXd RobotInterfaceFranka::_read_once_smooth_initial_positions(const double& samples)
{
    _check_if_robot_is_connected();
    VectorXd q = VectorXd::Zero(7);
    _update_status_message("Reading smooth initial joints...",verbose_);
    for (int i=0;i<samples;i++ )
    {
        franka::RobotState state = robot_sptr_->readOnce();
        q = Map<VectorXd>(state.q_d.data(), 7);
        q = _compute_recursive_mean(samples, q);
    }
    return q;
}


/**
 * @brief robot_driver_franka::read_once_initial_positions
 * @return
 */
VectorXd RobotInterfaceFranka::read_once_initial_positions()
{
    _check_if_robot_is_connected();
    VectorXd q = VectorXd::Zero(7);
    _update_status_message("Reading initial joints...",verbose_);
    franka::RobotState state = robot_sptr_->readOnce();
    q = Map<VectorXd>(state.q_d.data(), 7);
    return q;
}


/**
 * @brief robot_driver_franka::move_robot_to_target_joint_positions
 * @param q_target
 */
void RobotInterfaceFranka::move_robot_to_target_joint_positions(const VectorXd& q_target)
{
    _finish_echo_robot_state();
    _check_if_robot_is_connected();

    if (!initialize_flag_)
    {
        throw std::runtime_error(std::string("Error in set_initial_robot_configuration. Driver not initialized.  Use initialize(); "));
    }
    MotionGenerator motion_generator(speed_factor_joint_position_controller_, q_target);

    try {
        _update_status_message("Moving robot...", verbose_);
        robot_sptr_->control(motion_generator);
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
    }

    _start_echo_robot_state_mode_thread();

}



/**
 * @brief robot_driver_franka::set_joint_target_positions
 * @param set_target_joint_positions_rad
 */
void RobotInterfaceFranka::set_target_joint_positions(const VectorXd& set_target_joint_positions_rad)
{
    desired_joint_positions_ = set_target_joint_positions_rad;
}


/**
 * @brief robot_driver_franka::get_joint_target_positions
 * @return
 */
VectorXd RobotInterfaceFranka::get_joint_target_positions()
{
    return desired_joint_positions_;
}


/**
 * @brief robot_driver_franka::read_gripper
 * @param gripper_state
 * @return
 */
double RobotInterfaceFranka::read_gripper(const GRIPPER_STATES& gripper_state)
{
   _check_if_hand_is_connected();
   franka::GripperState state = gripper_sptr_->readOnce();
   switch (gripper_state) {
   case RobotInterfaceFranka::GRIPPER_STATES::WIDTH:
       return state.width;
       break;
   case RobotInterfaceFranka::GRIPPER_STATES::MAX_WIDTH:
       return state.max_width;
       break;
   default:
       throw std::runtime_error(std::string("Wrong argument in sas_robot_driver_franka::read_gripper. "));
       break;
   }
}

/**
 * @brief robot_driver_franka::read_grasped_status
 * @return
 */
bool RobotInterfaceFranka::read_grasped_status()
{
    _check_if_hand_is_connected();
    franka::GripperState state = gripper_sptr_->readOnce();
    return state.is_grasped;
}


/**
 * @brief robot_driver_franka::set_gripper
 * @param width
 */
void RobotInterfaceFranka::set_gripper(const double& width)
{
    _check_if_hand_is_connected();
    auto gripper_max_width = read_gripper(RobotInterfaceFranka::GRIPPER_STATES::MAX_WIDTH);
    if (width > gripper_max_width)
    {
       throw std::runtime_error(
           std::string("You used a width = ") +
           std::to_string(width) +
           std::string(". Maximum width allowed is ") +
           std::to_string(gripper_max_width)
           );
    }
    gripper_sptr_->move(width, speed_gripper_);
}


/**
 * @brief robot_driver_franka::gripper_homing
 */
void RobotInterfaceFranka::gripper_homing()
{
   _check_if_hand_is_connected();
   gripper_sptr_->homing();
}


/**
 * @brief robot_driver_franka::get_home_robot_configuration
 * @return
 */
VectorXd RobotInterfaceFranka::get_home_robot_configuration()
{
    return q_home_configuration_;
}


