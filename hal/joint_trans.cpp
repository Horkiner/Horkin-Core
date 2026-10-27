// 传动换算：q = s * θ / r + q0，力按功率守恒乘回 r
#include "hal/joint_trans.hpp"

#include <cmath>
#include <stdexcept>

namespace horkin
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        void check_ratio(const JointTrans& t)
        {
            if (!(t.ratio > 0.0))
            {
                throw std::runtime_error("joint_trans ratio must be > 0");
            }
        }
    }

    double lead_to_ratio(double lead)
    {
        if (!(lead > 0.0))
        {
            throw std::runtime_error("lead_to_ratio lead must be > 0");
        }
        return 2.0 * kPi / lead;
    }

    double motor_to_joint_pos(double motor_pos, const JointTrans &t)
    {
        check_ratio(t);
        return t.sign * motor_pos / t.ratio + t.offset;
    }

    double motor_to_joint_vel(double motor_vel, const JointTrans &t)
    {
        check_ratio(t);
        return t.sign * motor_vel / t.ratio;
    }

    double motor_to_joint_tau(double motor_tau, const JointTrans& t)
    {
        check_ratio(t);
        return t.sign * motor_tau * t.ratio;
    }

    double joint_to_motor_pos(double q, const JointTrans &t)
    {
        check_ratio(t);
        return t.sign * (q - t.offset) * t.ratio;
    }

    double joint_to_motor_vel(double dq, const JointTrans& t)
    {
        check_ratio(t);
        return t.sign * dq * t.ratio;
    }    

    double joint_to_motor_tau(double tau, const JointTrans& t)
    {
        check_ratio(t);
        return t.sign * tau / t.ratio;
    }

    double joint_to_motor_gain(double k, const JointTrans& t)
    {
        check_ratio(t);
        return k / (t.ratio * t.ratio);
    }
}