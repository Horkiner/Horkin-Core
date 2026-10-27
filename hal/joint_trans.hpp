// 关节 ↔ 电机输出轴；导程、减速比只出现在参数里
#pragma once

namespace horkin 
{
    /**
     * 单轴传动参数
     *
     * 成员
     * - ratio：关节走 1 个单位，电机输出轴转多少 rad
     *          转动轴直连 = 1；中间还有减速再乘上那一级
     *          丝杠 = 2π / 导程(m)
     * - sign：电机方向与 URDF 关节方向相同为 +1，相反为 -1
     * - offset：电机读数为 0 时的关节位置（rad 或 m）
     */
    struct JointTrans
    {
        double ratio{1.0};
        double sign{1.0};
        double offset{0.0};
    };

    /**
     * 丝杠导程 → ratio
     *
     * 参数
     * - lead：米/转，必须 > 0
     *
     * 返回：2π / lead
     */
    double lead_to_ratio(double lead);

    /** 电机输出轴 → 关节 */
    double motor_to_joint_pos(double motor_pos, const JointTrans& t);
    double motor_to_joint_vel(double motor_vel, const JointTrans& t);
    double motor_to_joint_tau(double motor_tau, const JointTrans& t);

    /**
     * 关节 → 电机输出轴
     *
     * 说明
     * - 增益要除以 ratio²：力矩除一次 ratio，位置误差再乘一次
     * - 现在只读用不到增益，Write 时必须用
     */
    double joint_to_motor_pos(double q, const JointTrans& t);
    double joint_to_motor_vel(double dq, const JointTrans& t);
    double joint_to_motor_tau(double tau, const JointTrans& t);
    double joint_to_motor_gain(double k, const JointTrans& t); 
}