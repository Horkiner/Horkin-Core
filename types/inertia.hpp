// 关节空间惯性矩阵
#pragma once

#include "types/joint_layout.hpp"

#include <array>

namespace horkin
{
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