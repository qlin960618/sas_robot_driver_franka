#include "sas_robot_driver_coppelia.h"


namespace sas
{


RobotDriverCoppelia::RobotDriverCoppelia(const RobotDriverCoppeliaConfiguration &configuration, std::atomic_bool *break_loops):
RobotDriver(break_loops),
    configuration_(configuration),
    robot_mode_(configuration.robot_mode),
    jointnames_(configuration.jointnames),
    mirror_mode_(configuration.mirror_mode),
    dim_configuration_space_(configuration.jointnames.size()),
    real_robot_topic_prefix_(configuration.real_robot_topic_prefix)
{
    vi_ = std::make_shared<DQ_VrepInterface>();
    desired_joint_velocities_ = VectorXd::Zero(dim_configuration_space_);
    auto nodehandle = ros::NodeHandle();
    std::cout<<"Rostopic: "<<"/"+real_robot_topic_prefix_<<std::endl;
    franka1_ros_ = std::make_shared<sas::RobotDriverInterface>(nodehandle, "/"+real_robot_topic_prefix_);
}

VectorXd RobotDriverCoppelia::get_joint_positions()
{
    return current_joint_positions_;
}

void RobotDriverCoppelia::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    desired_joint_positions_ = desired_joint_positions_rad;
}

VectorXd RobotDriverCoppelia::get_joint_velocities()
{
    return current_joint_velocities_;
}

void RobotDriverCoppelia::set_target_joint_velocities(const VectorXd &desired_joint_velocities)
{
    desired_joint_velocities_ = desired_joint_velocities;
}

VectorXd RobotDriverCoppelia::get_joint_forces()
{
    return current_joint_forces_;
}

RobotDriverCoppelia::~RobotDriverCoppelia()=default;

void RobotDriverCoppelia::connect()
{

    vi_->connect(configuration_.ip, configuration_.port, 500, 10);
    vi_->set_joint_target_velocities(jointnames_, VectorXd::Zero(dim_configuration_space_));
    std::cout<<"Connecting..."<<std::endl;
}

void RobotDriverCoppelia::disconnect()
{
    vi_->disconnect();
    if (joint_velocity_control_mode_thread_.joinable())
    {
        joint_velocity_control_mode_thread_.join();
    }
    if (joint_velocity_control_mirror_mode_thread_.joinable())
    {
        joint_velocity_control_mirror_mode_thread_.join();
    }
    std::cout<<"Disconnected."<<std::endl;
}

void RobotDriverCoppelia::initialize()
{
    vi_->start_simulation();
    if (mirror_mode_ == false)
    {
        _start_joint_velocity_control_thread();
    }else{
        _start_joint_velocity_control_mirror_thread();
    }
    std::cout<<"Velocity loop running..."<<std::endl;
}

void RobotDriverCoppelia::deinitialize()
{
    vi_->set_joint_target_velocities(jointnames_, VectorXd::Zero(dim_configuration_space_));
    vi_->stop_simulation();
    finish_motion();
    std::cout<<"Deinitialized."<<std::endl;
}




void RobotDriverCoppelia::_update_robot_state(const VectorXd &q, const VectorXd &q_dot, const VectorXd &forces)
{
    current_joint_positions_  = q;
    current_joint_velocities_ = q_dot;
    current_joint_forces_     =  forces;
}

void RobotDriverCoppelia::finish_motion()
{
    for (int i=0;i<1000;i++)
    {
        set_target_joint_positions(current_joint_positions_);
        set_target_joint_velocities(VectorXd::Zero(dim_configuration_space_));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    finish_motion_ = true;
}

void RobotDriverCoppelia::_start_joint_velocity_control_mode()
{
    try{
        finish_motion_ = false;
        VectorXd q = vi_->get_joint_positions(jointnames_);
        VectorXd q_dot =  vi_->get_joint_velocities(jointnames_);
        VectorXd forces = vi_->get_joint_torques(jointnames_);
        _update_robot_state(q, q_dot, forces);

        desired_joint_positions_ = q;
        while(true)
        {

            VectorXd q = vi_->get_joint_positions(jointnames_);
            VectorXd q_dot =  vi_->get_joint_velocities(jointnames_);
            VectorXd forces = vi_->get_joint_torques(jointnames_);
            _update_robot_state(q, q_dot, forces);


            if (robot_mode_  == std::string("VelocityControl"))
            {              
                    vi_->set_joint_target_velocities(jointnames_, desired_joint_velocities_); 
            }
            else if (robot_mode_  == std::string("PositionControl"))
            {
                    vi_->set_joint_target_positions(jointnames_, desired_joint_positions_);
            }
            if (finish_motion_) {
                    finish_motion_ = false;
                    return;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"sas_robot_coppelia_driver::_start_joint_velocity_control_mode(). Error or exception caught " << e.what()<<std::endl;
    }
    catch(...)
    {
        std::cout<<"sas_robot_coppelia_driver::_start_joint_velocity_control_mode(). Error or exception caught " <<std::endl;
    }


}

void RobotDriverCoppelia::_start_joint_velocity_control_thread()
{
    finish_motion_ = false;
    if (joint_velocity_control_mode_thread_.joinable())
    {
        joint_velocity_control_mode_thread_.join();
    }
    if (joint_velocity_control_mirror_mode_thread_.joinable())
    {
        joint_velocity_control_mirror_mode_thread_.join();
    }
    joint_velocity_control_mode_thread_ = std::thread(&RobotDriverCoppelia::_start_joint_velocity_control_mode, this);
}

void RobotDriverCoppelia::_start_joint_velocity_control_mirror_thread()
{
    finish_motion_ = false;
    if (joint_velocity_control_mode_thread_.joinable())
    {
        joint_velocity_control_mode_thread_.join();
    }
    if (joint_velocity_control_mirror_mode_thread_.joinable())
    {
        joint_velocity_control_mirror_mode_thread_.join();
    }
    joint_velocity_control_mirror_mode_thread_ = std::thread(&RobotDriverCoppelia::_start_joint_velocity_control_mirror_mode, this);
}

void RobotDriverCoppelia::_start_joint_velocity_control_mirror_mode()
{

    try{
        finish_motion_ = false;
        std::cout<<"Waiting for real robot topics..."<<std::endl;
        VectorXd q;
        while (ros::ok()) {
            if (franka1_ros_->is_enabled())
            {
                    q = franka1_ros_->get_joint_positions();
                    break;
            }
            ros::spinOnce();
        }
        std::cout<<"Done!"<<std::endl;

        VectorXd q_vrep = vi_->get_joint_positions(jointnames_);
        VectorXd q_dot_vrep =  vi_->get_joint_velocities(jointnames_);
        VectorXd forces_vrep = vi_->get_joint_torques(jointnames_);
        _update_robot_state(q_vrep, q_dot_vrep, forces_vrep);

        desired_joint_positions_ = q_vrep;


        while(ros::ok())
        {
            q = franka1_ros_->get_joint_positions();
            if (q.size() == dim_configuration_space_)
            {
                VectorXd q_vrep = vi_->get_joint_positions(jointnames_);
                VectorXd q_dot_vrep =  vi_->get_joint_velocities(jointnames_);
                VectorXd forces_vrep = vi_->get_joint_torques(jointnames_);
                _update_robot_state(q_vrep, q_dot_vrep, forces_vrep);


                if (robot_mode_  == std::string("VelocityControl"))
                {
                    vi_->set_joint_target_velocities(jointnames_, gain_*(q-q_vrep));
                }
                else if (robot_mode_  == std::string("PositionControl"))
                {
                        vi_->set_joint_target_positions(jointnames_, q);
                }
                if (finish_motion_) {
                        finish_motion_ = false;
                        return;
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cout<<"sas_robot_coppelia_driver::_start_joint_velocity_control_mirror_mode(). Error or exception caught " << e.what()<<std::endl;
    }
    catch(...)
    {
        std::cout<<"sas_robot_coppelia_driver::_start_joint_velocity_control_mirror_mode(). Error or exception caught " <<std::endl;
    }


}




} // sas namespace
