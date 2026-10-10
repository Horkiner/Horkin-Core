// 机械臂关节类型
#pragma once

#include <array>
#include <cstddef>

namespace horkin
{
    /**
     * 关节运动副类型
     *
     * 成员
     * - Revolute：转动，逻辑量 rad / rad/s / N·m
     * - Prismatic：移动，逻辑量 m / m/s / N
     * - Fixed：固定，不占 q
     */
    enum class JointKind
    {
        Revolute,
        Prismatic,
        Fixed
    };

    /**
     * 逻辑可动关节个数
     *
     * 说明
     * - 固定关节不计入
     * - 换构型时改这一处，并同步 URDF
     */
    inline constexpr std::size_t kNJoints = 6;

    /**
     * 每个逻辑关节的运动副，下标 i 对应第 i 轴
     *
     * 说明
     * - 换构型时改本表和 URDF
     * - 算法按表查询，不写死某一轴
     */
    inline constexpr std::array<JointKind, kNJoints> kJointKind = {
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Prismatic,
        JointKind::Revolute,
        JointKind::Revolute,
        JointKind::Revolute,
    };
}
