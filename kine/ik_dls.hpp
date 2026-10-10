#pragma once

#include "model/robot_model.hpp"
#include "types/joint.hpp"
#include "types/pose.hpp"

#include <array>

namespace horkin 
{
    /**
     * 末端一拍笛卡尔增量
     *
     * 成员（下标）
     * - [0..2]：vx, vy, vz，单位 m
     * - [3..5]：wx, wy, wz，单位 rad
     *
     * 说明
     * - 坐标系与 jacobian() 一致
     * - 当作速度时再乘周期
     */
    using Twist = std::array<double, 6>;

    /**
     * ik_step 的阻尼与量纲权重
     *
     * 成员
     * - damping：λ，越大越不敢在奇异位形大动
     * - ori_length：m/rad，角速度行换成米后再做列缩放
     */
    struct IkParams
    {
        double damping{1e-2};
        double ori_length{1.0};
    };

    /**
     * ik_pose 的收敛与步长
     *
     * 成员
     * - step：每拍 ik_step 的阻尼与姿态权重
     * - pos_tol：位置收敛阈值，米
     * - ori_tol：姿态收敛阈值，弧度
     * - max_lin：每拍最大平移，米
     * - max_ang：每拍最大转动，弧度
     * - max_iter：最大迭代次数，超过则失败
     */
    struct IkPoseParams
    {
        IkParams step{};
        double pos_tol{1e-4};
        double ori_tol{1e-3};
        double max_lin{0.02};
        double max_ang{0.05};
        int max_iter{40};
    };

    /**
    * 单步 DLS，建议给力控 / 实时周期用
    *
    * 流程：在当前 q 上解 J dq = twist，不改 q，也不迭代到位姿
    *
    * 参数
    * - model：运动学模型
    * - q：当前关节（转动 rad，移动 m）
    * - twist：末端这一拍增量 [vx,vy,vz,wx,wy,wz]
    * - params：阻尼 λ、姿态权重 ori_length
    *
    * 返回
    * 关节增量 dq，单位与 q 相同，调用方自己做 q += dq
    */
    JointVec ik_step(const RobotModel& model, const JointVec& q,
                     const Twist& twist, const IkParams& params = {});
    

    /**
    * 多步 DLS，建议给规划 / 求精确解用
    *
    * 流程：误差 → 限步 → ik_step → 夹限位，直到进阈值或超迭代
    *
    * 参数
    * - model：运动学模型
    * - q：种子关节；成功时被改成解
    * - target：目标末端位姿
    * - params：阈值、每拍步长、最大迭代
    *
    * 返回
    * - 成功：true，解写进 q
    * - 失败：false，q 停在最后一次迭代
    */
    bool ik_pose(const RobotModel& model, JointVec& q, const Pose& target,
                 const IkPoseParams& params = {});
}