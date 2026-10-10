// 关节数据类型
#pragma once

#include "joint_layout.hpp"
#include "types/joint_layout.hpp"

namespace horkin
{
    /**
    * 关节空间向量，长度 kNJoints
    *
    * 说明
    * - 每个可动关节一个分量，固定关节不占位
    * - 下标 i 是第 i 轴，运动副见 kJointKind[i]
    * - 装位置时：转动 rad，移动 m。速度和力用同一下标，单位见 JointState
    */
    using JointVec = std::array<double, kNJoints>;

    /**
     * 一拍关节状态，来自 IJointDriver::Read
     *
     * 成员
     * - q：位置（rad 或 m）
     * - dq：速度（rad/s 或 m/s）
     * - tau：力矩或力（N·m 或 N）
     * - valid：false 时忽略其余字段
     */
    struct JointState
    {
        JointVec q{};
        JointVec dq{};
        JointVec tau{};
        bool valid{false};
    };
}
