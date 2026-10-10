#pragma once

#include "types/time.hpp"
#include <array>

namespace horkin
{
    /**
     * 六维向量
     *
     * 说明
     * - 力旋量布局 [fx, fy, fz, mx, my, mz]
     */
    using Vector6 = std::array<double, 6>;

    /**
     * 力旋量
     *
     * 成员
     * - ft：[fx, fy, fz, mx, my, mz]，单位 N、N·m
     *
     * 说明
     * - 离开 signal/ 之后视为 TCP 系、已补偿
     * - 端口读到的是传感器系原始力
     */
    struct Wrench
    {
        Vector6 ft{};
    };

    /**
     * 一拍六维力采样
     *
     * 成员
     * - wrench：力旋量
     * - t_acq：采集时刻，单调时钟 ns
     * - valid：false 时忽略其余字段
     */
    struct WrenchSample
    {
        Wrench wrench{};
        TimeNs t_acq{0};
        bool valid{false};
    };
}
