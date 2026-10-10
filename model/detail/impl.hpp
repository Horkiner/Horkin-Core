// RobotModel 的 Pinocchio 实体。只给 model / kine / dyn 的 .cpp 包含。
#pragma once

#include "model/robot_model.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>

#include <cstddef>

namespace horkin
{
    /**
     * Pinocchio 模型与算法工作区
     *
     * 说明
     * - data 会被 fk、jacobian、以及随后的动力学算法改写
     * - 同一实例上顺序调用即可；不要在两个线程里同时用一个 RobotModel
     * - 结果都按值拷出，不把 data 里的引用交给调用方
     */
    struct RobotModel::Impl
    {
        pinocchio::Model model;
        mutable pinocchio::Data data;
        pinocchio::FrameIndex ee{0};  // 末端坐标系
        JointVec q_lower{};
        JointVec q_upper{};
    };

    namespace detail
    {
        /**
         * 关节位置：JointVec → Pinocchio 的 q
         *
         * 说明
         * - 外部是 std::array，Pinocchio 要 Eigen::VectorXd
         * - 下标与逻辑关节一致，长度用 model.nq
         */
        inline Eigen::VectorXd to_qpin(const pinocchio::Model& model, const JointVec& q)
        {
            Eigen::VectorXd qpin(model.nq);
            for (int i = 0; i < model.nq; ++i)
            {
                qpin[i] = q[static_cast<std::size_t>(i)];
            }
            return qpin;
        }

        /**
         * 关节速度或加速度：JointVec → Pinocchio 的 v / a
         *
         * 说明
         * - 长度用 model.nv
         * - 本臂每个可动关节 1 自由度，nv 与 nq、kNJoints 相同
         */
        inline Eigen::VectorXd to_vpin(const pinocchio::Model& model, const JointVec& v)
        {
            Eigen::VectorXd vpin(model.nv);
            for (int i = 0; i < model.nv; ++i)
            {
                vpin[i] = v[static_cast<std::size_t>(i)];
            }
            return vpin;
        }

        /**
         * 关节力：Pinocchio 的 tau → JointVec
         *
         * 说明
         * - 转动关节 N·m，移动关节 N
         * - 构造时已保证 nv == kNJoints
         */
        inline JointVec from_tau(const Eigen::VectorXd& tau)
        {
            JointVec out{};
            for (std::size_t i = 0; i < kNJoints; ++i)
            {
                out[i] = tau[static_cast<int>(i)];
            }
            return out;
        }
    }
}
