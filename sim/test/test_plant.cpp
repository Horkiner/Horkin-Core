#include "model/robot_model.hpp"
#include "sim/plant.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

namespace
{
    double max_abs(const horkin::JointVec& a)
    {
        double m = 0.0;
        for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        {
            const double v = std::abs(a[i]);
            if (v > m) m = v;
        }
        return m;
    }

    // 各关节 |a - b| 的最大值
    double max_abs_diff(const horkin::JointVec& a, const horkin::JointVec& b)
    {
        double m = 0.0;
        for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        {
            const double v = std::abs(a[i] - b[i]);
            if (v > m) m = v;
        }
        return m;
    }
}

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");
    const double dt = 1e-3;  // 1 ms
    const int nstep = 500;   // 0.5 s

    // 每拍用当前重力托住，应几乎静止
    {
        horkin::Plant plant(robot);
        const horkin::JointVec q0 = plant.q();
        for (int k = 0; k < nstep; ++k)
        {
            plant.step(robot.gravity(plant.q()), dt);
        }
        if (max_abs_diff(plant.q(), q0) > 1e-2)
        {
            std::fprintf(stderr, "gravity hold still moving |dq|=%.4e\n",
                         max_abs(plant.dq()));
            return 1;
        }
    }

    // 零力矩应在重力下倒下，q 必须明显变化
    {
        horkin::Plant plant(robot);
        const horkin::JointVec q0 = plant.q();
        horkin::JointVec zero{};
        for (int k = 0; k < nstep; ++k)
        {
            plant.step(zero, dt);
        }
        if (max_abs_diff(plant.q(), q0) < 1e-4)
        {
            std::fprintf(stderr, "zero torque did not move the arm\n");
            return 1;
        }
    }

    std::printf("test_plant ok\n");
    return 0;
}