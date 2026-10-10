#include "kine/ik_dls.hpp"
#include "model/robot_model.hpp"
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

    std::size_t revolute = 0;
    for (; revolute < horkin::kNJoints; ++revolute)
    {
        if (horkin::kJointKind[revolute] == horkin::JointKind::Revolute)
        {
            break;
        }
    }
    q[revolute] = 0.15;

    if (!horkin::ik_pose(robot, q, target))
    {
        std::fprintf(stderr, "ik_pose failed\n");
        return 1;
    }

    const horkin::Pose back = robot.fk(q);
    const double dp = std::hypot(back.position.x - target.position.x,
                                 back.position.y - target.position.y,
                                 back.position.z - target.position.z);
    if (dp > 1e-3)
    {
        std::fprintf(stderr, "ik_pose |dp|=%.4e\n", dp);
        return 1;
    }
    std::fprintf(stderr, "test_ik ok  |dp|=%.4e\n", dp);
    return 0;
}