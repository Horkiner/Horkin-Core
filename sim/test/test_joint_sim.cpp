#include "model/robot_model.hpp"
#include "sim/joint_sim.hpp"
#include "types/command.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");
    const double dt = 1e-3;
    horkin::JointSim joints(robot, dt);

    horkin::JointState st{};
    joints.Read(st);
    if (st.valid)
    {
        std::fprintf(stderr, "Read vaild before Enable\n");
        return 1;
    }

    joints.Enable();
    joints.Read(st);
    const horkin::JointVec q0 = st.q;

    horkin::JointCommand cmd{};
    for (int k = 0; k < 500; ++k)
    {
        joints.Read(st);  // 将仿真器中的数据读到 st
        cmd.tau_ff = robot.gravity(st.q);
        joints.Write(cmd);
    }
    joints.Read(st);
    if (!st.valid)
    {
        std::fprintf(stderr, "Read invalid after Enable");
        return 1;
    }

    double dp = 0.0;
    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
    {
        dp += std::abs(st.q[i] - q0[i]);
    }
    if (dp > 1e-3)
    {
        std::fprintf(stderr, "JointSim gravity hold drifted dp=%.4e\n", dp);
        return 1;
    }

    std::printf("test_joint_sim ok\n");
    return 0;
}