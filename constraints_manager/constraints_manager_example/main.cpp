#include <iostream>
#include "constraints_manager.h"

using namespace Eigen;

int main()
{
    auto manager = ConstraintsManager(3);
    auto A1 = MatrixXd::Zero(3,3);
    auto b1 = VectorXd::Zero(3);
    auto A2 = MatrixXd::Ones(1,3);
    auto b2 = VectorXd::Ones(1);

    manager.add_inequality_constraint(A1, b1);
    manager.add_inequality_constraint(A2, b2);

    MatrixXd A;
    VectorXd b;

    std::tie(A,b) = manager.get_inequality_constraints();
    std::cout<<"A: "<<std::endl;
    std::cout<<A<<std::endl;
    std::cout<<"b: "<<std::endl;
    std::cout<<b<<std::endl;

}
