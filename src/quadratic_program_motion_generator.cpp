#include "quadratic_program_motion_generator.h"


QuadraticProgramMotionGenerator::QuadraticProgramMotionGenerator(const double &speed_factor,
                                                                 const Eigen::VectorXd &q_initial,
                                                                 const Eigen::VectorXd &q_dot_initial,
                                                                 const Eigen::VectorXd &q_goal)
{
    n_links_ = _get_dimensions(q_initial, q_dot_initial, q_goal);
    solver_ = std::make_unique<DQ_QPOASESSolver>(DQ_QPOASESSolver());
    constraints_manager_ = std::make_unique<ConstraintsManager>(ConstraintsManager(n_links_));
    _check_gains();
    q_ = q_initial;
    q_dot_ = q_dot_initial;

    H_ = MatrixXd::Identity(n_links_, n_links_);
    lb_ = VectorXd::Zero(n_links_);
    ub_ = VectorXd::Zero(n_links_);
    I_ = MatrixXd::Identity(n_links_, n_links_);

    _check_speed_factor(speed_factor);
    speed_factor_ = speed_factor;

    q_dot_max_  *= speed_factor_;
    q_dot_dot_max_ *= speed_factor_;

    K1_ = N1_.asDiagonal();
    K2_ = N2_.asDiagonal();
}

QuadraticProgramMotionGenerator::QuadraticProgramMotionGenerator(const double &speed_factor, const VectorXd &q_dot_initial, const VectorXd &q_dot_goal)
{
    n_links_ = _get_dimensions(q_dot_initial, q_dot_initial, q_dot_goal);
    solver_ = std::make_unique<DQ_QPOASESSolver>(DQ_QPOASESSolver());
    constraints_manager_ = std::make_unique<ConstraintsManager>(ConstraintsManager(n_links_));
    _check_gains();
    q_dot_ = q_dot_initial;

    H_ = MatrixXd::Identity(n_links_, n_links_);
    lb_ = VectorXd::Zero(n_links_);
    ub_ = VectorXd::Zero(n_links_);
    I_ = MatrixXd::Identity(n_links_, n_links_);

    _check_speed_factor(speed_factor);
    speed_factor_ = speed_factor;

    q_dot_max_  *= speed_factor_;
    q_dot_dot_max_ *= speed_factor_;

    K1_ = N1_.asDiagonal();
    K2_ = N2_.asDiagonal();
}


void QuadraticProgramMotionGenerator::_check_speed_factor(const double& speed_factor) const
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

int QuadraticProgramMotionGenerator::_get_dimensions(const Eigen::VectorXd &q1,
                                                     const Eigen::VectorXd &q2,
                                                     const Eigen::VectorXd &q3) const
{
    _check_sizes(q1, q2, q3);
    return q1.size();
}


void QuadraticProgramMotionGenerator::set_gains(const double& n1, const double& n2)
{
    n1_ = n1;
    n2_ = n2;
    _check_gains() ;
    N1_ <<n1_, n1_, n1_, n1_, n1_, n1_, n1_;
    N2_ <<n2_, n2_, n2_, n2_, n2_, n2_, n2_;
    K1_ = N1_.asDiagonal();
    K2_ = N2_.asDiagonal();
}

void QuadraticProgramMotionGenerator::set_diagonal_gains(const VectorXd& K1, const VectorXd& K2)
{
    K1_ = K1.asDiagonal();
    K2_ = K2.asDiagonal();
}

void QuadraticProgramMotionGenerator::_check_sizes(const Eigen::VectorXd &q1,
                                                   const Eigen::VectorXd &q2,
                                                   const Eigen::VectorXd &q3) const
{
    if (q1.size() != q2.size() or q1.size() != q3.size() or q2.size() != q3.size())
    {
        throw std::runtime_error(std::string("Wrong sizes in vectors. "));
    }
}

void QuadraticProgramMotionGenerator::_check_gains()
{
    if ( std::pow(n1_, 2) - 4*n2_ <= 0)
    {
        throw std::runtime_error(std::string("Wrong gains!!!. You need to use gains such that n1^2 - 4*n2 > 0. "));
    }
}


VectorXd QuadraticProgramMotionGenerator::compute_new_configuration(const VectorXd& q_goal,
                                                                    const double& T)
{

    f_ = K1_*(q_dot_ + K2_*(q_-q_goal)); //f_ = n1_*(q_dot_ + n2_*(q_-q_goal));
    for (int i=0; i<n_links_;i++)
    {
          lb_[i] =   std::max( -q_dot_dot_max_[i], (1/T)*(-q_dot_max_[i] - q_dot_[i]));
          ub_[i] =   std::min(  q_dot_dot_max_[i], (1/T)*( q_dot_max_[i] - q_dot_[i]));
    }


    constraints_manager_->add_inequality_constraint(-I_, -lb_);
    constraints_manager_->add_inequality_constraint( I_,  ub_);


    std::tie(A_, b_) = constraints_manager_->get_inequality_constraints();

    auto u = solver_->solve_quadratic_program(H_, f_, A_, b_, Aeq_, beq_);
    q_ = q_ +T*q_dot_ +0.5*std::pow(T,2)*u;
    q_dot_ = q_dot_ + T*u;

    return q_;
}

VectorXd QuadraticProgramMotionGenerator::compute_new_configuration_velocities(const VectorXd &q_dot_goal, const double &T)
{

    f_ = 2*K1_*(q_dot_-q_dot_goal);
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

