#include "sim/joint_sim.hpp"
#include "model/robot_model.hpp"
#include "sim/plant.hpp"
#include "types/command.hpp"
#include "types/joint.hpp"

#include <stdexcept>

namespace horkin
{
    JointSim::JointSim(const RobotModel& model, double dt, const JointVec& q0)
        : plant_(model, q0), dt_(dt)
    {
        if (!(dt_ > 0.0))
        {
            throw std::runtime_error("JointSim dt must be > 0");
        }
    }

    void JointSim::Enable() { enabled_ = true; }
    void JointSim::Disable() { enabled_ = false; }

    void JointSim::Read(JointState& out)
    {
        out.q = plant_.q();
        out.dq = plant_.dq();
        out.tau = last_tau_;
        out.valid = enabled_;
    }

    void JointSim::Write(const JointCommand& cmd)
    {
        if (!enabled_)
        {
            return;
        }
        last_tau_ = cmd.tau_ff;
        plant_.step(cmd.tau_ff, dt_);
    }
}