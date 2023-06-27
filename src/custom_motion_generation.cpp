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


#include "custom_motion_generation.h"

/**
 * @brief CustomMotionGeneration::CustomMotionGeneration
 */
CustomMotionGeneration::CustomMotionGeneration(const double &speed_factor,
                                               const Eigen::VectorXd &q_initial,
                                               const Eigen::VectorXd &q_dot_initial,
                                               const Eigen::VectorXd &q_goal)
{
    n_links_ = _get_dimensions(q_initial, q_dot_initial, q_goal);
    solver_ = std::make_unique<DQ_QPOASESSolver>();
    constraints_manager_ = std::make_unique<ConstraintsManager>(n_links_);
    q_ = q_initial;
    q_dot_ = q_dot_initial;
    q_prior_ = q_;

    H_  = MatrixXd::Identity(n_links_, n_links_);
    lb_ = VectorXd::Zero(n_links_);
    ub_ = VectorXd::Zero(n_links_);
    I_  = MatrixXd::Identity(n_links_, n_links_);

    _check_speed_factor(speed_factor);
    speed_factor_ = speed_factor;

    q_dot_max_  *= speed_factor_;
    q_dot_dot_max_ *= speed_factor_;
}


/**
 * @brief CustomMotionGeneration::CustomMotionGeneration
 * @param speed_factor
 * @param q_dot_initial
 * @param q_dot_goal
 */
CustomMotionGeneration::CustomMotionGeneration(const double &speed_factor, const VectorXd &q_dot_initial, const VectorXd &q_dot_goal)
{
    n_links_ = _get_dimensions(q_dot_initial, q_dot_goal, q_dot_goal);
    solver_ = std::make_unique<DQ_QPOASESSolver>();
    constraints_manager_ = std::make_unique<ConstraintsManager>(n_links_);
    q_dot_ = q_dot_initial;


    H_  = MatrixXd::Identity(n_links_, n_links_);
    lb_ = VectorXd::Zero(n_links_);
    ub_ = VectorXd::Zero(n_links_);
    I_  = MatrixXd::Identity(n_links_, n_links_);

    _check_speed_factor(speed_factor);
    speed_factor_ = speed_factor;

    q_dot_max_  *= speed_factor_;
    q_dot_dot_max_ *= speed_factor_;

}

/**
 * @brief CustomMotionGeneration::set_proportional_gain
 * @param gain
 */
void CustomMotionGeneration::set_proportional_gain(const double &gain)
{
    gain_ = gain;
}

/**
 * @brief CustomMotionGeneration::_check_sizes
 * @param q1
 * @param q2
 * @param q3
 */
void CustomMotionGeneration::_check_sizes(const Eigen::VectorXd &q1,
                                          const Eigen::VectorXd &q2,
                                          const Eigen::VectorXd &q3) const
{
    if (q1.size() != q2.size() or q1.size() != q3.size())
    {
        throw std::runtime_error(std::string("Wrong sizes in vectors. "));
    }
}

/**
 * @brief CustomMotionGeneration::_get_dimensions
 * @param q1
 * @param q2
 * @param q3
 * @return
 */
int CustomMotionGeneration::_get_dimensions(const Eigen::VectorXd &q1,
                                                     const Eigen::VectorXd &q2,
                                                     const Eigen::VectorXd &q3) const
{
    _check_sizes(q1, q2, q3);
    return q1.size();
}



/**
 * @brief CustomMotionGeneration::_check_speed_factor
 * @param speed_factor
 */
void CustomMotionGeneration::_check_speed_factor(const double& speed_factor) const
{
    if (speed_factor > 1.0)
    {
        throw std::runtime_error("Speed factor must be <= 1.0");
    }
    if (speed_factor <= 0.0)
    {
        throw std::runtime_error("Speed factor must be > 0.0");
    }
}


/**
 * @brief CustomMotionGeneration::compute_new_configuration
 * @param q_goal
 * @param T
 * @return
 */
VectorXd CustomMotionGeneration::compute_new_configuration(const VectorXd &q_goal, const double &T)
{
    f_ = 2*gain_*(q_-q_goal);
    for (int i=0; i<n_links_;i++)
    {

        lb_[i] = std::max({(1/T)*(q_min_[i]-q_[i]),
                           -q_dot_max_[i],
                           -sqrt((2*q_dot_dot_max_[i]*(q_[i]-q_min_[i]))),
                           q_dot_prior_[i] - T*q_dot_dot_max_[i]});

        ub_[i] = std::min({(1/T)*(q_max_[i]-q_[i]),
                            q_dot_max_[i],
                            sqrt((2*q_dot_dot_max_[i]*(q_max_[i]-q_[i]))),
                            q_dot_prior_[i] + T*q_dot_dot_max_[i]});
    }


    constraints_manager_->add_inequality_constraint(-I_, -lb_);
    constraints_manager_->add_inequality_constraint( I_,  ub_);


    std::tie(A_, b_) = constraints_manager_->get_inequality_constraints();

    auto u = solver_->solve_quadratic_program(H_, f_, A_, b_, Aeq_, beq_);

    VectorXd accel = (u-q_dot_prior_)*(1/T);

    for (int i=0; i<n_links_;i++)
    {
        if (std::abs(accel[i]) > ref_q_dot_dot_max_[i])
        {
            std::cout<<"Acceleration limit violation"<<std::endl;
        }
    }

    VectorXd vel = (q_-q_prior_)*(1/T);

    for (int i=0; i<n_links_;i++)
    {
        if (std::abs(vel[i]) > ref_q_dot_max_[i])
        {
            std::cout<<"Velocity limit violation "<<i<<std::endl;
        }
    }


    q_dot_prior_ = u;
    q_prior_ = q_;

    q_ = q_ + T*u;



    return q_;
}

/**
 * @brief CustomMotionGeneration::compute_new_configuration_velocities
 * @param q_dot_goal
 * @param T
 * @return
 */
VectorXd CustomMotionGeneration::compute_new_configuration_velocities(const VectorXd &q_dot_goal, const double &T)
{
    f_ = 2*gain_*(q_dot_-q_dot_goal);
    for (int i=0; i<n_links_;i++)
    {
        lb_[i] =   std::max( -q_dot_dot_max_[i], (1/T)*(-q_dot_max_[i] - q_dot_[i]));
        ub_[i] =   std::min(  q_dot_dot_max_[i], (1/T)*( q_dot_max_[i] - q_dot_[i]));
    }


    constraints_manager_->add_inequality_constraint(-I_, -lb_);
    constraints_manager_->add_inequality_constraint( I_,  ub_);


    std::tie(A_, b_) = constraints_manager_->get_inequality_constraints();

    auto u = solver_->solve_quadratic_program(H_, f_, A_, b_, Aeq_, beq_);
    q_dot_ = q_dot_ + T*u;

    return q_dot_;
}
