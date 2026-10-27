#include "hal/joint_trans.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

int main()
{
    // 直连转动轴：数应原样通过
    const horkin::JointTrans id{};
    if (std::abs(horkin::motor_to_joint_pos(1.2, id) - 1.2) > 1e-12 ||
        std::abs(horkin::motor_to_joint_vel(-0.5, id) + 0.5) > 1e-12 ||
        std::abs(horkin::motor_to_joint_tau(3.0, id) - 3.0) > 1e-12)
    {
        std::fprintf(stderr, "identity trans failed\n");
        return 1;
    }

    // 丝杆：1 转 = 0.144 m
    const double lead = 0.144;
    horkin::JointTrans j3;
    j3.ratio = horkin::lead_to_ratio(lead);
    const double two_pi = 2.0 * 3.14159265358979323846;
    if (std::abs(horkin::motor_to_joint_pos(two_pi, j3) - lead) > 1e-12)
    {
        std::fprintf(stderr, "lead pos wrong: %.8f\n",
                     horkin::motor_to_joint_pos(two_pi, j3));
        return 1;
    }

    // 往返：关节 0.10 m -> 关节
    const double q = 0.10;
    const double th = horkin::joint_to_motor_pos(q, j3);
    if (std::abs(horkin::motor_to_joint_pos(th, j3) - q) > 1e-12)
    {
        std::fprintf(stderr, "round trip pos failed\n");
        return 1;
    }

    // 功率守恒：电机 1 N·m → 关节力 = ratio
    if (std::abs(horkin::motor_to_joint_tau(1.0, j3) - j3.ratio) > 1e-12)
    {
        std::fprintf(stderr, "tau scale wrong\n");
        return 1;
    }

    // 反向：电机正转，关节位置变负
    horkin::JointTrans rev{2.0, -1.0, 0.5};
    if (std::abs(horkin::motor_to_joint_pos(2.0, rev) - (-1.0 + 0.5)) > 1e-12)
    {
        std::fprintf(stderr, "sign/offset pos wrong\n");
        return 1;
    }
    // kp=8、ratio=2 → 电机侧增益 2
    if (std::abs(horkin::joint_to_motor_gain(8.0, rev) - 2.0) > 1e-12)
    {
        std::fprintf(stderr, "gain scale wrong\n");
        return 1;
    }
    std::printf("test_joint_trans ok\n");
    return 0;
}