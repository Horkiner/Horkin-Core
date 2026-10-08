#include "kine/ik_dls.hpp"
#include "kine/robot_model.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"
#include "types/pose.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");
    const horkin::JointVec lower = robot.q_lower();
    const horkin::JointVec upper = robot.q_upper();
    for (std::size_t j = 0; j < horkin::kNJoints; ++j)
    {
        if (!(lower[j] < upper[j]))
        {
            std::fprintf(stderr, "joint %zu limit lower >= upper\n", j);
            return 1;
        }
    }

    horkin::JointVec q{};
    const horkin::Pose target = robot.fk(q);
}