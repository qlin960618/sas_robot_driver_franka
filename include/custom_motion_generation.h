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


#pragma once
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <memory>
#include "constraints_manager.h"


using namespace DQ_robotics;

class CustomMotionGeneration
{
private:
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    std::unique_ptr<DQ_QPOASESSolver> solver_;
    std::unique_ptr<ConstraintsManager> constraints_manager_;


    Vector7d q_dot_max_ = (Vector7d() << 2.0, 1.0, 1.5, 1.25, 3, 1.5, 3).finished();
    Vector7d q_dot_dot_max_ = (Vector7d() << 8, 8, 8, 8, 8, 8, 8).finished();
    const Vector7d q_max_ = ((Vector7d() <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished());
    const Vector7d q_min_ = ((Vector7d() << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished());
    int n_links_ = 7;

    const Vector7d ref_q_dot_max_ = (Vector7d() << 2.0, 1.0, 1.5, 1.25, 3, 1.5, 3).finished();
    const Vector7d ref_q_dot_dot_max_ = (Vector7d() << 8, 8, 8, 8, 8, 8, 8).finished();

    double speed_factor_;
    double gain_ = 1.0;

    VectorXd q_;
    VectorXd q_dot_;

    VectorXd q_dot_prior_ = VectorXd::Zero(7);
    VectorXd q_prior_;

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
    void _check_speed_factor(const double& speed_factor) const;



public:
    CustomMotionGeneration(const double &speed_factor,
                           const Eigen::VectorXd &q_initial,
                           const Eigen::VectorXd &q_dot_initial,
                           const Eigen::VectorXd &q_goal);

    CustomMotionGeneration(const double &speed_factor,
                           const Eigen::VectorXd &q_dot_initial,
                           const Eigen::VectorXd &q_dot_goal);

    void set_proportional_gain(const double& gain);

    VectorXd compute_new_configuration(const VectorXd& q_goal, const double& T);
    VectorXd compute_new_configuration_velocities(const VectorXd& q_dot_goal, const double& T);
};

