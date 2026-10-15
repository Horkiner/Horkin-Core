// MIT 式内环：把 JointCommand 变成关节力矩
#pragma once

#include "types/command.hpp"
#include "types/joint.hpp"

namespace horkin
{
    /**
     * 仿真伺服一拍
     *
     * 流程：tau = tau_ff + kp*(q_cmd - q) + kd*(dq_cmd - dq)
     *
     * 参数
     * - cmd：目标位置、速度、前馈、增益
     * - q / dq：植物当前状态
     *
     * 返回：关节力。转动 N·m，移动 N
     *
     * 说明
     * - kp=kd=0 时退化为只出 tau_ff，和现在的 JointSim::Write 一样
     * - 不积分；积分仍在 Plant::step
     */
     JointVec servo_mit(const JointCommand& cmd, const JointVec& q, const JointVec& dq);
}