// 机器人模型定义文件，对接 Pinocchio
#pragma once

#include "types/joint.hpp"
#include "types/pose.hpp"
#include "types/jacobian.hpp"

#include <memory>
#include <string>

namespace horkin 
{
    class RobotModel
    {
        public:
            // Load once at process start. Not for the RT loop.
            explicit RobotModel(const std::string& urdf_path,
                                const std::string& ee_frame = "tool");
            
            RobotModel(const RobotModel&) = delete;
            RobotModel& operator=(const RobotModel&) = delete;
            ~RobotModel();

            Pose fk(const JointVec& q) const;
            Jacobian jacobian(const JointVec& q) const;
            
            // 模型关节最小值限位，返回类型：JointVec
            JointVec q_lower() const;
            // 模型关节最大值限位，返回类型：JointVec
            JointVec q_upper() const;

        private:
            struct Impl;
            std::unique_ptr<Impl> impl_;  // 智能指针
    };
}