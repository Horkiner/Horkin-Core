#include "model/robot_model.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

namespace 
{
    bool near(const horkin::JointVec& a, const horkin::JointVec& b, double tol)
    {
        for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        {
            if (std::abs(a[i] - b[i]) > tol)
            {
                return false;
            }
        }
        return true;
    }
}

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");

    horkin::JointVec q{};
    horkin::JointVec zero{};
    const horkin::JointVec g0 = robot.gravity(q);
    if (!near(g0, robot.rnea(q, zero, zero), 1e-9))
    {
        std::fprintf(stderr, "gravity != rnea(q, 0, 0) at q=0\n");
        return 1;
    }

    q[4] = 0.4;
    const horkin::JointVec g1 = robot.gravity(q);
    if (near(g0, g1, 1e-6))
    {
        std::fprintf(stderr, "gravity did not change with pose\n");
        return 1;
    }
    if (!near(g1, robot.rnea(q, zero, zero), 1e-9))
    {
        std::fprintf(stderr, "gravity != rnea(q, 0, 0) at q=0\n");
        return 1;
    }

    horkin::JointVec dq{};
    dq[4] = 2.0;
    if (near(g1, robot.rnea(q, dq, zero), 1e-6))
    {
        std::fprintf(stderr, "rnea ignored joint velocity\n");
        return 1;
    }

    std::printf("test_rnea ok\n");
    return 0;
}