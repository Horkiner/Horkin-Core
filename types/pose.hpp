#pragma once

namespace horkin
{
    /** 三维向量，单位随使用处：位置为 m */
    struct Vec3
    {
        double x{};
        double y{};
        double z{};
    };

    /**
     * 3x3 旋转矩阵，行优先
     *
     * 成员
     * - x：第 0 行 R00 R01 R02
     * - y：第 1 行 R10 R11 R12
     * - z：第 2 行 R20 R21 R22
     */
    struct Rotation
    {
        Vec3 x{};
        Vec3 y{};
        Vec3 z{};
    };

    /**
    * 刚体位姿（位置 + 旋转）
    *
    * 成员
    * - position：原点坐标，单位 m
    * - rotation：3x3 行优先旋转矩阵，默认单位阵
    *
    * 说明
    * - 坐标系由填入方约定（例如 fk 给出基座系）
    * - 插值用四元数时在转换函数里做，不放进本类型
    */
    struct Pose
    {
        Vec3 position{};
        Rotation rotation{ {1, 0, 0}, 
                           {0, 1, 0}, 
                           {0, 0, 1} };
    };
}