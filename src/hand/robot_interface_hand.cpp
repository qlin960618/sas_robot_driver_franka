#include "robot_interface_hand.h"



RobotInterfaceHand::RobotInterfaceHand(const std::string &ROBOT_IP):ip_(ROBOT_IP)
{
    gripper_sptr_ = std::make_shared<franka::Gripper>(ip_);
}

/**
  * @brief robot_driver_franka::_check_if_hand_is_connected
  */
void RobotInterfaceHand::_check_if_hand_is_connected()
{
    if(!gripper_sptr_)
        throw std::runtime_error("Invalid hand pointer. You must connect(), then initialize(). "
                                 + std::string("Example:  robot_driver_franka(IP, robot_driver_franka::MODE::None, robot_driver_franka::HAND::ON)" ));
}


double RobotInterfaceHand::read_gripper(const GRIPPER_MODE_STATES& gripper_state)
{
    _check_if_hand_is_connected();
    franka::GripperState state = gripper_sptr_->readOnce();
    switch (gripper_state) {
    case RobotInterfaceHand::GRIPPER_MODE_STATES::WIDTH:
        return state.width;
        break;
    case RobotInterfaceHand::GRIPPER_MODE_STATES::MAX_WIDTH:
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
bool RobotInterfaceHand::read_grasped_status()
{
    _check_if_hand_is_connected();
    franka::GripperState state = gripper_sptr_->readOnce();
    return state.is_grasped;
}


/**
 * @brief robot_driver_franka::set_gripper
 * @param width
 */
void RobotInterfaceHand::set_gripper(const double& width)
{
    _check_if_hand_is_connected();
    auto gripper_max_width = read_gripper(RobotInterfaceHand::GRIPPER_MODE_STATES::MAX_WIDTH);
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
void RobotInterfaceHand::gripper_homing()
{
    _check_if_hand_is_connected();
    gripper_sptr_->homing();
}

bool RobotInterfaceHand::grasp_object(const double &width, const double &speed, const double &force, double epsilon_inner, double epsilon_outer)
{
    // Check for the maximum grasping width.
    _check_if_hand_is_connected();
    franka::GripperState gripper_state = gripper_sptr_->readOnce();
    if (gripper_state.max_width < width) {
        std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
        return false;
    }
    // Grasp the object.
    if (!gripper_sptr_->grasp(width, 0.1, 20)) {
        std::cout << "Failed to grasp object." << std::endl;
        return false;
    }else{
        return true;
    }


}

void RobotInterfaceHand::release_object()
{
    _check_if_hand_is_connected();
    franka::GripperState gripper_state = gripper_sptr_->readOnce();
    if(gripper_state.is_grasped)
    {
        gripper_sptr_->stop();
    }
}
