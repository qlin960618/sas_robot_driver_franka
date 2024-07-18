//
// Created by qlin on 7/17/24.
//

#include "sas_robot_driver_franka_hand.h"

namespace sas {

    //const static int SLAVE_MODE_JOINT_CONTROL;
    //const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    RobotDriverFrankaHand::~RobotDriverFrankaHand()
    {
        if(_is_connected())
        {
            disconnect();
        }
    }

    RobotDriverFrankaHand::RobotDriverFrankaHand(
        const RobotDriverFrankaHandConfiguration& configuration,
        const RobotDriverROSConfiguration& ros_configuration,
        std::atomic_bool* break_loops):
    configuration_(configuration), ros_configuration_(ros_configuration), break_loops_(break_loops)
    {
        gripper_sptr_ = nullptr;

    }

    bool RobotDriverFrankaHand::_is_connected() const
    {
        if(gripper_sptr_ == nullptr) return false;
        if(!gripper_sptr_) return false;
        else return true;
    }

    VectorXd RobotDriverFrankaHand::get_joint_positions()
    {
        return joint_positions_;

    }
    void RobotDriverFrankaHand::set_target_joint_positions(const VectorXd& desired_joint_positions_rad)
    {

    }

    VectorXd RobotDriverFrankaHand::get_joint_velocities()
    {
        return VectorXd::Zero(1);
    }
    void RobotDriverFrankaHand::set_target_joint_velocities(const VectorXd& desired_joint_velocities)
    {

    }

    VectorXd RobotDriverFrankaHand::get_joint_forces()
    {
        return VectorXd::Zero(1);
    }

    void RobotDriverFrankaHand::start_control_loop()
    {

        Clock clock = Clock(ros_configuration_.thread_sampling_time_nsec);
        clock.init();
        ROS_INFO_STREAM("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::start_control_loop::Starting control loop.");
        while(!(*break_loops_))
        {
            if(!_is_connected()) throw std::runtime_error("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::start_control_loop::Robot is not connected.");


            clock.update_and_sleep();
        }
        ROS_INFO_STREAM("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::start_control_loop::Exiting control loop.");

    }


    void RobotDriverFrankaHand::connect()
    {
        gripper_sptr_ = std::make_shared<franka::Gripper>(configuration_.robot_ip);
        if(!_is_connected()) throw std::runtime_error("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::connect::Could not connect to the robot.");

    }
    void RobotDriverFrankaHand::disconnect() noexcept
    {
        ROS_WARN_STREAM("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::disconnecting...");
        gripper_sptr_->~Gripper();
        gripper_sptr_ = nullptr;
    }

    /**
     * @brief robot_driver_franka::gripper_homing
     */
    void RobotDriverFrankaHand::gripper_homing()
    {
        if(!_is_connected()) throw std::runtime_error("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::gripper_homing::Robot is not connected.");
        auto ret = gripper_sptr_->homing();
        if(!ret)
        {
            throw std::runtime_error("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::gripper_homing::Failed to home the gripper.");
        }
        ROS_INFO_STREAM("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::gripper_homing::Gripper homed.");
    }

    void RobotDriverFrankaHand::initialize()
    {
        if(!_is_connected()) throw std::runtime_error("["+ ros::this_node::getName()+"]::[RobotDriverFrankaHand]::initialize::Robot is not connected.");
        gripper_homing();
    }

    void RobotDriverFrankaHand::deinitialize()
    {
        if(_is_connected())
        {
            franka::GripperState gripper_state = gripper_sptr_->readOnce();
            if(gripper_state.is_grasped)
            {
                gripper_sptr_->stop();
            }
        }
    }




} // sas