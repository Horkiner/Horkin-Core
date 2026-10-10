// RobotModel::jacobian，几何雅可比
#include "model/detail/impl.hpp"

#include "types/joint_layout.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <cstddef>

namespace horkin
{
    Jacobian RobotModel::jacobian(const JointVec& q) const
    {
        const Eigen::VectorXd qpin = detail::to_qpin(impl_->model, q);

        // model.nv 表示速度的维度，转/平动：1，球关节：3，浮动基：6
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, impl_->model.nv);
        J.setZero();  // Pinocchio 要先清零

        pinocchio::computeFrameJacobian(impl_->model, impl_->data, qpin,
                                      impl_->ee, pinocchio::LOCAL_WORLD_ALIGNED, J);

        Jacobian out{};
        for (std::size_t col = 0; col < kNJoints; ++col)
            for (std::size_t row = 0; row < 6; ++row)
                out[row][col] = J(static_cast<int>(row), static_cast<int>(col));

        return out;
    }
}
