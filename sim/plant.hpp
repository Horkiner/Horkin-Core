// 关节空间 plant: M ddq = tau - rnea(q, dq, 0)
#pragma once

#include "model/robot_model.hpp"
#include "types/joint.hpp"

namespace horkin
{
    /**
     * 无接触刚体积分器
     *
     * 说明
     * - 同一份 RobotModel，与真机 FK / 重力共用
     * - 不实现 IJointDriver；端口封装在 JointSim
     * - 一步半隐式欧拉，夹 URDF 限位
     */
    class Plant
    {
        public:
            /**
             * 参数
             * - model：已构造的刚体模型，必须比 Plant 活得长
             * - q0：初始位置，默认全 0
             */
            explicit Plant(const RobotModel& model, const JointVec& q0 = {});

            const JointVec& q() const;
            const JointVec& dq() const;

            /**
             * 直接改状态，给单测用
             *
             * 参数
             * - q / dq：新的位置、速度
             */
            void set_state(const JointVec& q, const JointVec& dq);


            /**
             * 积分一拍
             *
             * 流程：M(q) ddq = tau - rnea(q, dq, 0)，再半隐式欧拉，夹限位
             *
             * 参数
             * - tau：关节力（转动 N·m，移动 N）
             * - dt：周期，秒，必须 > 0
             */
            void step(const JointVec& tau, double dt);

        private:
            const RobotModel& model_;
            JointVec q_{};
            JointVec dq_{};
    };
}