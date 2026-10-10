// RobotModel::rnea，逆动力学
#include "model/detail/impl.hpp"
#include <pinocchio/algorithm/rnea.hpp>
namespace horkin
{
    JointVec RobotModel::rnea(const JointVec& q, const JointVec& dq,
                              const JointVec& ddq) const
    {
        const Eigen::VectorXd qpin = detail::to_qpin(impl_->model, q);
        const Eigen::VectorXd vpin = detail::to_vpin(impl_->model, dq);
        const Eigen::VectorXd apin = detail::to_vpin(impl_->model, ddq);
        const Eigen::VectorXd& tau =
            pinocchio::rnea(impl_->model, impl_->data, qpin, vpin, apin);
        return detail::from_tau(tau);
    }
}