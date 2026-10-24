#include "control/force/admittance_axis.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

int main()
{
    const double dt = 1e-3;
    const int nstep = 500;

    // k=d=0、常力：a=f/m，半隐式欧拉
    {
        horkin::AdmitAxisState s{};
        horkin::AdmitAxisParams p{};
        p.mass = 2.0;
        const double f = 4.0;
        const double a = f / p.mass;
        for (int k = 0; k < nstep; k++)
        {
            horkin::admittance_axis_step(s, f, dt, p);
        }
        // 以恒力持续 nstep 
        const double v_expect = nstep * a * dt;
        const double e_expect = a * dt * dt * nstep * (nstep + 1) / 2;
        if (std::abs(s.v - v_expect) > 1e-9 || std::abs(s.e - e_expect) > 1e-9)
        {
            std::fprintf(stderr, "free mass wrong e=%.6f v=%.6f\n", s.e, s.v);
            return 1;
        }
    }

    // f=0、有刚度：应从 e>0 往回拉
    {
        horkin::AdmitAxisState s{};
        s.e = 0.1;
        horkin::AdmitAxisParams p{};
        p.mass = 1.0;
        p.damping = 4.0;
        p.stiffness = 100.0;
        for (int k = 0; k < nstep; ++k)
        {
            horkin::admittance_axis_step(s, 0.0, dt, p);
        }
        if (!(s.e < 0.1))
        {
            std::fprintf(stderr, "stiffness did not pull back, e=%.4f", s.e);
            return 1;
        }
    }

    std::printf("test_admittance_axis ok\n");
    return 0;
}