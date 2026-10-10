// RobotModel::fk，正运动学
#include "model/detail/impl.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace horkin
{
    Pose RobotModel::fk(const JointVec& q) const
    {
        const Eigen::VectorXd qpin = detail::to_qpin(impl_->model, q);

        pinocchio::forwardKinematics(impl_->model, impl_->data, qpin);
        pinocchio::updateFramePlacements(impl_->model, impl_->data);

        const pinocchio::SE3& M = impl_->data.oMf[impl_->ee];
        const Eigen::Vector3d& t = M.translation();
        const Eigen::Matrix3d& R = M.rotation();

        Pose pose;
        pose.position = {t.x(), t.y(), t.z()};
        pose.rotation = {
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2),
        };
        return pose;
    }
}
