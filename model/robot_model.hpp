// 刚体模型。运动学函数体在 kine/，动力学函数体在 dyn/。
#pragma once

#include "types/joint.hpp"
#include "types/pose.hpp"
#include "types/jacobian.hpp"

#include <memory>
#include <string>

namespace horkin
{
    /**
     * 刚体模型：从 URDF 建一次，运动学和动力学都打在这份模型上
     *
     * 说明
     * - Pinocchio 只在 model/detail/impl.hpp，公开头不暴露
     * - 逻辑关节顺序与 kJointKind 一致
     * - 进程启动时构造一次，循环里调成员函数
     * - fk、jacobian 的函数体在 kine/
     * - 动力学成员（rnea、gravity）声明加在本类，函数体放在 dyn/
     */
    class RobotModel
    {
        public:
            /**
             * 从 URDF 加载模型，进程启动时调用一次，不要进实时循环
             *
             * 参数
             * - urdf_path：URDF 路径
             * - ee_frame：末端坐标系名，默认 tool
             */
            explicit RobotModel(const std::string& urdf_path,
                                const std::string& ee_frame = "tool");

            RobotModel(const RobotModel&) = delete;
            RobotModel& operator=(const RobotModel&) = delete;
            ~RobotModel();

            /**
             * 正运动学
             *
             * 流程：q → 末端位姿（基座系，参考点为 ee_frame）
             *
             * 参数
             * - q：当前关节
             *
             * 返回：Pose，position 为 m
             */
            Pose fk(const JointVec& q) const;

            /**
             * 几何雅可比
             *
             * 流程：在当前 q 上算末端 6×n，LOCAL_WORLD_ALIGNED
             * （原点线速度 + 世界系角速度）
             *
             * 参数
             * - q：当前关节
             *
             * 返回：Jacobian，J[行][关节]
             */
            Jacobian jacobian(const JointVec& q) const;

            /**
             * URDF 位置下限
             *
             * 返回：JointVec，单位与 q 相同
             */
            JointVec q_lower() const;

            /**
             * URDF 位置上限
             *
             * 返回：JointVec，单位与 q 相同
             */
            JointVec q_upper() const;
            
            /**
            * 逆动力学
            *
            * 流程：tau = M(q) ddq + c(q, dq) + g(q)
            *
            * 参数
            * - q：位置（转动 rad，移动 m）
            * - dq：速度（rad/s 或 m/s）
            * - ddq：加速度（rad/s² 或 m/s²）
            *
            * 返回：关节力。转动 N·m，移动 N
            */
            JointVec rnea(const JointVec& q, const JointVec& dq, const JointVec& ddq) const;
            
            /**
            * 重力项
            *
            * 流程：静止时托住机械臂的关节力，等于 rnea(q, 0, 0)
            *
            * 参数
            * - q：当前位置
            *
            * 返回：与 rnea 相同。前馈用 tau_ff = gravity(q)
            */
            JointVec gravity(const JointVec& q) const;

        private:
            struct Impl;
            std::unique_ptr<Impl> impl_;
    };
}
