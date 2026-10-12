# 换设备续写惯性矩阵 mass(q)

本文保存「RNEA 跑完后下一步做什么」那一轮的逐步说明，含全部示例代码。摩擦和仿真积分这次先不写。

新设备打开本仓库后，把下面整段贴进新的 Cursor 对话。离开前把本文件和未提交代码一起提交并推送。

---

继续 Horkin Core 的动力学，仓库是 horkin-core。rnea / gravity 已通过 test_rnea。下一步做惯性矩阵 mass(q)，我要自己手敲。你先读现有文件，缺什么就告诉我该写哪一段；我贴出代码后你再对照、纠错，并在我同意后帮我编译跑测试。不要重写已经写好的 rnea / gravity / 运动学，不要做 friction 或仿真积分。

已完成，不要推倒：

- RobotModel 在 model/。公开头 model/robot_model.hpp 不暴露 pinocchio。
- Pinocchio 的 Model 和 Data 在 model/detail/impl.hpp。只有 model/、kine/、dyn/ 的 cpp 能包含它。
- 转换函数：detail::to_qpin、detail::to_vpin、detail::from_tau。
- 运动学：kine/fk.cpp、kine/jacobian.cpp、kine/ik_dls.cpp。
- 动力学：dyn/rnea.cpp、dyn/gravity.cpp。test_rnea 已过。
- 构造函数读 config/urdf/v6_mc.urdf，末端帧名 tool。kNJoints = 6，下标 2 是移动副（米），其余转动（弧度）。
- 姿态扰动不要用 q[0]。joint1 轴与重力平行，绕它转不改变势能。gravity 测试已改成 q[4] = 0.4（joint5）。mass 测试同样用 q[4]。

方程：

```
tau = M(q) ddq + c(q, dq) + g(q)
```

rnea 给出整个 tau，gravity 给出 g。仿真要解 ddq，缺的是 M(q)。它是 n×n，装不进 JointVec，所以要先加公开类型。

## 1. types/inertia.hpp

仿 types/jacobian.hpp，新建（不要把 Inertia 塞进 jacobian.hpp）：

```cpp
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
```

horkin_types 是头文件库，不用改 CMake。非线性项 c(q,dq) 仍用 JointVec，不要为它再加类型。

## 2. 公开声明

model/robot_model.hpp 加上：

```cpp
#include "types/inertia.hpp"
```

类注释里「动力学成员」补上 mass。gravity() 后面、private: 前面加：

```cpp
/**
 * 关节空间惯性矩阵
 *
 * 流程：在当前 q 上算 M(q)，对称
 *
 * 参数
 * - q：当前位置
 *
 * 返回：Inertia，M[i][j]
 */
Inertia mass(const JointVec& q) const;
```

## 3. dyn/mass.cpp

```cpp
// RobotModel::mass，关节空间惯性矩阵
#include "model/detail/impl.hpp"

#include "types/joint_layout.hpp"

#include <pinocchio/algorithm/crba.hpp>

#include <cstddef>

namespace horkin
{
    Inertia RobotModel::mass(const JointVec& q) const
    {
        const Eigen::VectorXd qpin = detail::to_qpin(impl_->model, q);
        pinocchio::crba(impl_->model, impl_->data, qpin);

        // crba 只填上三角，下三角要自己对称拷过去
        const Eigen::MatrixXd& M = impl_->data.M;
        Inertia out{};
        for (std::size_t i = 0; i < kNJoints; ++i)
        {
            for (std::size_t j = i; j < kNJoints; ++j)
            {
                const double v = M(static_cast<int>(i), static_cast<int>(j));
                out[i][j] = v;
                out[j][i] = v;
            }
        }
        return out;
    }
}
```

必须包含 model/detail/impl.hpp，否则看不见 Impl。用 pinocchio::crba，不要用 crbaMinimal。返回值是 data.M 的引用，立刻拷进 Inertia，不要把 Eigen 矩阵传出函数。

Pinocchio 2.7 的文档写明：crba 只算上三角。不补下三角的话，M[j][i]（j > i）是未定义值，后面积分会错。

## 4. CMake

dyn/CMakeLists.txt 的源文件列表加上 mass.cpp：

```cmake
add_library(horkin_dyn STATIC
  rnea.cpp
  gravity.cpp
  mass.cpp
)
```

测试段在 test_rnea 旁边加 test_mass，不要改已经能过的 test_rnea：

```cmake
  add_executable(test_mass test/test_mass.cpp)
  target_link_libraries(test_mass PRIVATE horkin_dyn)
  target_compile_definitions(test_mass PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")
```

不要加入 friction.cpp。

## 5. dyn/test/test_mass.cpp

测四件事。姿态扰动继续用 q[4] = 0.4，不要用 q[0]。

```cpp
#include "model/robot_model.hpp"
#include "types/inertia.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");

    horkin::JointVec q{};
    const horkin::Inertia M0 = robot.mass(q);

    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
    {
        if (M0[i][i] <= 0.0)
        {
            std::fprintf(stderr, "M[%zu][%zu] <= 0\n", i, i);
            return 1;
        }
        for (std::size_t j = i + 1; j < horkin::kNJoints; ++j)
        {
            if (std::abs(M0[i][j] - M0[j][i]) > 1e-12)
            {
                std::fprintf(stderr, "M not symmetric\n");
                return 1;
            }
        }
    }

    q[4] = 0.4;
    const horkin::Inertia M1 = robot.mass(q);
    double diff = 0.0;
    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        for (std::size_t j = 0; j < horkin::kNJoints; ++j)
            diff += std::abs(M1[i][j] - M0[i][j]);
    if (diff < 1e-6)
    {
        std::fprintf(stderr, "mass did not change with pose\n");
        return 1;
    }

    // rnea(q, 0, ddq) 应等于 M ddq + gravity(q)
    horkin::JointVec ddq{};
    ddq[4] = 1.0;
    const horkin::JointVec g = robot.gravity(q);
    const horkin::JointVec tau = robot.rnea(q, horkin::JointVec{}, ddq);
    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
    {
        double mddq = 0.0;
        for (std::size_t j = 0; j < horkin::kNJoints; ++j)
            mddq += M1[i][j] * ddq[j];
        if (std::abs(tau[i] - (mddq + g[i])) > 1e-6)
        {
            std::fprintf(stderr, "M ddq + g != rnea at joint %zu\n", i);
            return 1;
        }
    }

    std::printf("test_mass ok\n");
    return 0;
}
```

第四组把 mass 和已经过的 rnea / gravity 钉在同一条方程上。JointVec{} 就是全零速度。

## 6. 编译

```bash
source scripts/env.sh
cmake -S . -B build -DHORKIN_BUILD_TESTS=ON
cmake --build build --target test_mass test_rnea test_fk test_ik
./build/dyn/test_mass
./build/dyn/test_rnea
```

注意选项是 HORKIN_BUILD_TESTS，中间有 S。写成 HORKIN_BUILD_TEST=ON 不会新开测试；能编出测试是因为仓库里这个选项已经开过。

约定：

- 文件名继续小写加下划线。注释用中文，风格跟 model/robot_model.hpp、dyn/gravity.cpp 一样。
- 成员函数是 const。impl_->data 是共用工作区，结果必须按值拷出。
- 依赖：kine 和 dyn 都只依赖 model，dyn 不依赖 kine。
- Pinocchio 只用 2.7。编译前 source scripts/env.sh。

mass 过了之后，下一步才是仿真 S0：用 M 和 rnea(q, dq, 0) 做关节空间积分，还没有接触。摩擦仍然先不做。
