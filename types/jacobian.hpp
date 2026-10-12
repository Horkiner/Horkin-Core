// 雅可比数据类型
#pragma once

#include "types/joint_layout.hpp"
#include "types/wrench.hpp"

#include <array>

namespace horkin
{
    /**
     * 几何雅可比：6 行末端速度，每列一个可动关节
     *
     * 成员（下标）
     * - 第一维：行，[vx, vy, vz, wx, wy, wz]
     * - 第二维：列，逻辑关节
     *
     * 说明
     * - 坐标系由 jacobian() 约定
     * - 取系数用 J[行][关节]
     */
    using Jacobian = std::array<std::array<double, kNJoints>, 6>;

    /**
     * 关节空间惯性矩阵 M(q)，n×n，对称
     *
     * 成员（下标）
     * - 第一维、第二维都是逻辑关节
     *
     * 说明
     * - 取系数用 M[i][j]
     * - 转动–转动：kg·m²；移动–移动：kg；交叉项：kg·m
     */
     using Inertia = std::array<std::array<double, kNJoints>, kNJoints>;

}
