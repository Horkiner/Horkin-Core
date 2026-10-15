// servo_mit，MIT 式内环
#include "sim/servo_sim.hpp"

#include "types/command.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cstddef>

namespace horkin
{
    JointVec servo_mit(const JointCommand& cmd, const JointVec &q, const JointVec &dq)
    {
        JointVec tau{};
        for (std::size_t i = 0; i < kNJoints; ++i)
        {
            tau[i] = cmd.tau_ff[i]
                   + cmd.kp[i] * (cmd.q[i] - q[i])
                   + cmd.kd[i] * (cmd.dq[i] - dq[i]);
        }
        return tau;
    }
}