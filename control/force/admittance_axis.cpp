// admittance_axis_step，单轴导纳半隐藏式欧拉
#include "control/force/admittance_axis.hpp"

#include <stdexcept>

namespace horkin
{
    void admitance_axis_step(AdmitAxisState &state, double f, double dt, 
                             const AdmitAxisParams &params)
    {
        if (!(dt > 0.0))
        {
            throw std::runtime_error("admittance_axis_step dt must be > 0");
        }
        if (!(params.mass > 0.0))
        {
            throw std::runtime_error("admittance_axis_step mass must be > 0");
        }

        const double a = (f - params.damping * state.v
                            - params.stiffness * state.e) / params.mass;
        state.v += a * dt;
        state.e += state.v * dt;
    }
}