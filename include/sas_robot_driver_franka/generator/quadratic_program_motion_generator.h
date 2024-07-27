#pragma once
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <memory>
#include "constraints_manager.h"


using namespace DQ_robotics;

class QuadraticProgramMotionGenerator
{
private:

    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;

    double speed_factor_;
    VectorXd q_;
    VectorXd q_dot_;

    int n_links_ = 7;
    Vector7d q_dot_max_ = (Vector7d() << 2.0, 1.0, 1.5, 1.25, 3, 1.5, 3).finished();
    Vector7d q_dot_dot_max_ = (Vector7d() << 8, 8, 8, 8, 8, 8, 8).finished();
    double n2_ = 1.0;
    double n1_ = 2000*std::sqrt(n2_);

    VectorXd N1_ = (VectorXd(7)<<n1_, n1_, n1_, n1_, n1_, n1_, n1_).finished();
    VectorXd N2_ = (VectorXd(7)<<n2_, n2_, n2_, n2_, n2_, n2_, n2_).finished();

    MatrixXd K1_;
    MatrixXd K2_;

    MatrixXd H_;
    VectorXd lb_;
    VectorXd ub_;
    MatrixXd A_;
    VectorXd b_;
    VectorXd f_;
    MatrixXd Aeq_;
    VectorXd beq_;
    MatrixXd I_;

    void   _check_sizes(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2, const Eigen::VectorXd &q3) const;
    int _get_dimensions(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2, const Eigen::VectorXd &q3) const;
    void _check_gains();
    std::unique_ptr<DQ_QPOASESSolver> solver_;
    std::unique_ptr<ConstraintsManager> constraints_manager_;

    void _check_speed_factor(const double& speed_factor) const;


public:
    QuadraticProgramMotionGenerator(const double &speed_factor,
                                    const Eigen::VectorXd &q_initial,
                                    const Eigen::VectorXd &q_dot_initial,
                                    const Eigen::VectorXd &q_goal);

    QuadraticProgramMotionGenerator(const double &speed_factor,
                                    const Eigen::VectorXd &q_dot_initial,
                                    const Eigen::VectorXd &q_dot_goal);

   VectorXd compute_new_configuration(const VectorXd& q_goal, const double& T);
   VectorXd compute_new_configuration_velocities(const VectorXd& q_dot_goal, const double& T);

   void set_gains(const double& n1, const double& n2);

   void set_diagonal_gains(const VectorXd& K1, const VectorXd& K2);

};


