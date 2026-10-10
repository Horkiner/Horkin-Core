#pragma once

#include "types/joint.hpp"

namespace horkin
{
    enum class JointCyclicMode
    {
        MIT,
        CSP,
        CSV,
        CST
    };

    /**
     * 一拍关节指令，交给 IJointDriver::Write
     *
     * 成员
     * - q / dq：位置、速度前馈
     * - tau_ff：力矩前馈
     * - kp / kd：MIT 式增益
     *
     * 说明
     * - 单位与 JointVec 相同
     * - 用不到的通道保持 0
     */
    struct JointCommand
    {
        JointVec q{};
        JointVec dq{};
        JointVec tau_ff{};
        JointVec kp{};
        JointVec kd{};
    };
}
