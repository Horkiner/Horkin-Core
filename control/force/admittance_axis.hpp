// 单轴导纳，半隐式欧拉。不含 6D、不含 IK
#pragma once

namespace horkin
{
    /**
     * 单轴导纳内部状态
     *
     * 成员
     * - e：位移（m 或 rad，由调用方约定）
     * - v：速度（m/s 或 rad/s）
     */
    struct AdmitAxisState
    {
        double e{0.0};
        double v{0.0};
    };

    /**
     * 单轴导纳参数
     *
     * 成员
     * - mass：等效质量，必须 > 0
     * - damping：阻尼
     * - stiffness：刚度；力跟踪时常为 0
     */
    struct AdmitAxisParams
    {
        double mass{1.0};
        double damping{0.0};
        double stiffness{0.0};
    };
    
    /**
     * 导纳一拍，改 state
     *
     * 流程：a = (f - d*v - k*e) / m，再 v += a*dt，e += v*dt
     *
     * 参数
     * - state：本轴状态，函数内更新
     * - f：本轴力（N 或 N·m）
     * - dt：周期，秒，必须 > 0
     * - params：m、d、k
     */
    void admitance_axis_step(AdmitAxisState& state, double f, double dt,
                              const AdmitAxisParams& params);
}