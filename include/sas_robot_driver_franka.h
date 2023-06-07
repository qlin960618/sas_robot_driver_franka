#pragma once


#include <exception>
#include <tuple>
#include <atomic>
#include <vector>
#include <memory>

#include <dqrobotics/DQ.h>

#include <sas_robot_driver/sas_robot_driver.h>
#include "robot_interface_franka.h"

using namespace DQ_robotics;
using namespace Eigen;

namespace sas
{


struct RobotDriverFrankaConfiguration
{
    std::string ip_address;
    int port;
    double speed;
};


class RobotDriverFranka: public RobotDriver
{
private:
    RobotDriverFrankaConfiguration configuration_;

    std::shared_ptr<RobotInterfaceFranka> robot_driver_interface_sptr_ = nullptr;

    //Joint positions
    VectorXd joint_positions_;
    //VectorXd joint_velocities_;
    VectorXd end_effector_pose_;
    std::vector<double> joint_positions_buffer_;
    std::vector<double> end_effector_pose_euler_buffer_;
    std::vector<double> end_effector_pose_homogenous_transformation_buffer_;

    //DQ _homogenous_vector_to_dq(const VectorXd& homogenousvector) const;
    //VectorXd _dq_to_homogenous_vector(const DQ& pose) const;
    //VectorXd _get_end_effector_pose_homogenous_transformation();
    //double _sign(const double &a) const;

  //  void _connect(); //Throws std::runtime_error()

   // void _motor_on(); //Throws std::runtime_error()
   // void _motor_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

   // void _set_speed(const float& speed, const float& acceleration, const float& deacceleration); //Throws std::runtime_error()

   // void _slave_mode_on(int mode); //Throws std::runtime_error()
    //void _slave_mode_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

public:
    const static int SLAVE_MODE_JOINT_CONTROL;
    const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    RobotDriverFranka(const RobotDriverFranka&)=delete;
    RobotDriverFranka()=delete;
    ~RobotDriverFranka();

    RobotDriverFranka(const RobotDriverFrankaConfiguration& configuration, std::atomic_bool* break_loops);


    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    VectorXd get_joint_velocities() override;
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities) override;

    VectorXd get_joint_forces() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    bool set_end_effector_pose_dq(const DQ& pose);
    DQ get_end_effector_pose_dq();
    
};
}
