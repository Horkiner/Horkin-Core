// RobotModel::gravity，重力项
#include "model/detail/impl.hpp"
#include "model/robot_model.hpp"
#include "types/joint.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <pinocchio/algorithm/rnea.hpp>

namespace horkin
{
    JointVec RobotModel::gravity(const JointVec& q) const
    {
        const Eigen::VectorXd qpin = detail::to_qpin(impl_->model, q);
        const Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, qpin);

        return detail::from_tau(g);
    }
}