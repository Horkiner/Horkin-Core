// RobotModel::model，关节空间惯量矩阵
#include "model/detail/impl.hpp"

#include "model/robot_model.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <pinocchio/algorithm/crba.hpp>

#include <cstddef>

namespace horkin
{
    Inertia RobotModel::mass(const JointVec& q) const
    {
        const Eigen::VectorXd qpin = detail::to_qpin(impl_->model, q);
        pinocchio::crba(impl_->model, impl_->data, qpin);

        // crba 只填上三角，下三角要自己对称拷过去
        const Eigen::MatrixXd& M = impl_->data.M;
        Inertia out{};
        for (std::size_t i = 0; i < kNJoints; ++i)
        {
            for (std::size_t j = i; j < kNJoints; ++j)
            {
                const double v = M(static_cast<int>(i), static_cast<int>(j));
                out[i][j] = v;
                out[j][i] = v;
            }
        }
        return out;
    }
}