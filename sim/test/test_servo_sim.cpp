#include "sim/servo_sim.hpp"
#include "types/command.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

int main()
{
    horkin::JointCommand cmd{};
    horkin::JointVec q{};
    horkin::JointVec dq{};

    // kp=kd=0，应原样返回 tau_ff
    cmd.tau_ff[0] = 1.5;
    horkin::JointVec tau = horkin::servo_mit(cmd, q, dq);
    if (std::abs(tau[0] - 1.5) > 1e-12)
    {
        std::fprintf(stderr, "servo ignored tau_ff\n");
        return 1;
    }
    for (std::size_t i = 1; i < horkin::kNJoints; ++i)
    {
        if (std::abs(tau[i]) > 1e-12)
        {
            std::fprintf(stderr, "servo leaked onto joint %zu\n", i);
            return 1;
        }
    }

    // 只有 kp:tau = kp * (q_cmd - q)
    cmd = {};
    cmd.q[0] = 0.4;
    cmd.kp[0] = 10.0;
    q[0] = 0.1;
    tau = horkin::servo_mit(cmd, q, dq);
    if (std::abs(tau[0] - 3.0) > 1e-12)
    {
        std::fprintf(stderr, "servo kp term wrong, got %.4f\n", tau[0]);
        return 1;
    } 

    std::printf("test_servo_sim ok\n");
    return 0;
}