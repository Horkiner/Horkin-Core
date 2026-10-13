#include "model/robot_model.hpp"
#include "types/inertia.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");

    horkin::JointVec q{};
    const horkin::Inertia M0 = robot.mass(q);

    // 检测是否为上三角矩阵
    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
    {
        if (M0[i][i] <= 0)
        {
            std::fprintf(stderr, "M[%zu][%zu] <= 0\n", i, i);
            return 1;
        }
        for (std::size_t j = i + 1; j < horkin::kNJoints; ++j)
        {
            if (std::abs(M0[i][j] - M0[j][i]) > 1e-12)
            {
                std::fprintf(stderr, "M not symmetric\n");
                return 1;
            }
        }
    }

    // 检测姿态变化惯性矩阵是否跟着变
    q[4] = 0.4;
    const horkin::Inertia M1 = robot.mass(q);
    double diff = 0.0;
    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        for (std::size_t j = 0; j < horkin::kNJoints; ++j)
            diff += std::abs(M1[i][j] - M0[i][j]);
    if (diff < 1e-6)
    {
        std::fprintf(stderr, "mass did not change with pose\n");
        return 1;
    }

    // rnea(q, 0, ddq) 应等于 M ddq + gravite(q)
    horkin::JointVec ddq{};
    ddq[4] = 1.0;
    const horkin::JointVec g = robot.gravity(q);
    const horkin::JointVec tau = robot.rnea(q, horkin::JointVec{}, ddq);
    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
    {
        double mddq = 0.0;
        for (std::size_t j = 0; j < horkin::kNJoints; ++j)
        {
            mddq += M1[i][j] * ddq[j];
        }
        if (std::abs(tau[i] - (mddq + g[i])) > 1e-6)
        {
            std::fprintf(stderr, "M ddq + g != rnea at joint %zu\n", i);
            return 1;
        }
    }

    std::printf("test_mass ok\n");
    return 0;
}
