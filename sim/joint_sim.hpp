// 仿真关节驱动，实现 IJointDriver
#pragma once

#include "model/robot_model.hpp"
#include "ports/joint_driver.hpp"
#include "sim/plant.hpp"
#include "types/command.hpp"
#include "types/joint.hpp"

namespace horkin
{
    /**
     * 仿真后端：Write 收 tau_ff，内部积分 plant，Read 回 q、dq
     *
     * 说明
     * - Enable 之前 Write 不积分，Read 的 valid=false
     */
    class JointSim : public IJointDriver
    {
        public:
            /**
             * 参数
             * - model：刚体模型，必须比 JointSim 活得长
             * - dt：积分周期，秒
             * - q0：初始关节
             */ 
            explicit JointSim(const RobotModel& model, double dt, 
                              const JointVec& q0 = {});

            void Enable() override;
            void Disable() override;
            void Read(JointState& out) override;
            void Write(const JointCommand& cmd) override;
        
        private:
            Plant plant_;
            double dt_;
            bool enabled_{false};
            JointVec last_tau_{};
    };
}