// Plant::step，关节空间半隐式欧拉
#include "sim/plant.hpp"

#include "model/robot_model.hpp"
#include "types/inertia.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <stdexcept>

namespace horkin
{
    namespace 
    {
        Eigen::MatrixXd to_matrix(const Inertia& M)
        {
            Eigen::MatrixXd out(static_cast<int>(kNJoints), static_cast<int>(kNJoints));
            for (std::size_t i = 0; i < kNJoints; ++i)
                for (std::size_t j = 0; j < kNJoints; ++j)
                    out(static_cast<int>(i), static_cast<int>(j)) = M[i][j];
            return out;
        }

        Eigen::VectorXd to_eigen(const JointVec& v)
        {
            Eigen::VectorXd out(static_cast<int>(kNJoints));
            for (std::size_t i = 0; i < kNJoints; ++i)
                out[static_cast<int>(i)] = v[i];
            return out;
        }

        JointVec from_eigen(const Eigen::VectorXd& v)
        {
            JointVec out{};
            for (std::size_t i = 0; i < kNJoints; ++i)
            {
                out[i] = v[static_cast<int>(i)];
            }
            return out;
        }
    }

    Plant::Plant(const RobotModel&model, const JointVec& q0)
        : model_(model), q_(q0)
    {}

    const JointVec& Plant::q() const { return q_; }
    const JointVec& Plant::dq() const { return dq_; }

    void Plant::set_state(const JointVec& q, const JointVec& dq)
    {
        q_ = q;
        dq_ = dq;
    }

    void Plant::step(const JointVec& tau, double dt)
    {
        if(!(dt > 0.0))
        {
            throw std::runtime_error("Plant::step dt must be > 0");
        }

        const Inertia M = model_.mass(q_);
        const JointVec nle = model_.rnea(q_, dq_, JointVec{});

        Eigen::VectorXd rhs = to_eigen(tau) - to_eigen(nle);
        const Eigen::VectorXd ddq = to_matrix(M).ldlt().solve(rhs);

        // 半隐式欧拉
        dq_ = from_eigen(to_eigen(dq_) + ddq * dt);
        q_ = from_eigen(to_eigen(q_) + to_eigen(dq_) * dt);

        // 限位，超限之后速度都设 0
        const JointVec lower = model_.q_lower();
        const JointVec upper = model_.q_upper();
        for (std::size_t i = 0; i < kNJoints; ++i)
        {
            if (q_[i] < lower[i])
            {
                q_[i] = lower[i];
                dq_[i] = 0.0;
            }
            else if (q_[i] > upper[i]) 
            {
                q_[i] = upper[i];
                dq_[i] = 0.0;
            }
        }
    }
}   