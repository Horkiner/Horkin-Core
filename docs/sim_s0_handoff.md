# 换设备续写仿真 S0（关节空间积分）

`test_mass` 已通过。刚体动力学（rnea / gravity / mass）先停。下一步是仿真植物：没有接触、没有力传感器、不模拟 MIT 的 kp/kd。摩擦也不做。

离开前把本文件和未提交代码一起提交并推送。新设备打开仓库后，把下面「继续 Horkin Core」那一整段贴进新的 Cursor 对话。按步骤一次只写一个文件，贴给 Cursor 对照后再编。

---

继续 Horkin Core 的仿真 S0，仓库是 horkin-core。我要自己手敲。你先读现有文件和 docs/sim_s0_handoff.md。每次只告诉我下一步该写哪一个文件；我贴出代码后你再对照、纠错，并在我同意后帮我编译跑测试。不要重写 kine / dyn / model，不要做 contact、sensor_sim、world、friction、导纳。

已完成，不要推倒：

- RobotModel 在 model/。公开头不暴露 pinocchio。动力学成员：rnea、gravity、mass。test_rnea、test_mass 已过。
- 运动学：fk、jacobian、ik_dls。test_fk、test_ik 已过。
- URDF：config/urdf/v6_mc.urdf，末端帧 tool。kNJoints = 6，下标 2 是移动副（米），其余转动（弧度）。
- 姿态扰动不要用 q[0]（绕重力轴）。gravity / mass 测试用的是 q[4]。
- ports：IJointDriver（Enable / Disable / Read / Write），Write 吃 JointCommand。
- sim/ 现在还是 INTERFACE 占位，没有任何 cpp。

S0 方程（正向积分）：

```
M(q) ddq = tau - rnea(q, dq, 0)
```

rnea(q, dq, 0) 就是 c(q,dq) + g(q)。tau 这一步当成直接加在关节上的力（转动 N·m，移动 N）。半隐式欧拉：先更新速度，再用新速度更新位置。

```
ddq = M^{-1} (tau - rnea(q, dq, 0))
dq  += ddq * dt
q   += dq * dt
```

然后用 robot.q_lower() / q_upper() 夹 q；夹到边上的轴把 dq 置 0。

分两层，不要合成一个文件：

| 文件 | 职责 |
|------|------|
| sim/plant.hpp/.cpp | 持有 q、dq 和 const RobotModel&；step(tau, dt) 积分 |
| sim/joint_sim.hpp/.cpp | 实现 IJointDriver；Write 只取 tau_ff 调 plant.step；Read 回 q、dq |

Plant 不是端口，内部类。JointSim 才是可换后端。S0 的 Write 忽略 cmd.q / dq / kp / kd。6×6 求解只在 plant.cpp 里用 Eigen，公开头只出 JointVec。不要包含 model/detail/impl.hpp，不要包含 pinocchio 头。sim 通过 robot.mass / robot.rnea 调用动力学。

步骤顺序：1 plant.hpp → 2 plant.cpp + CMake → 3 test_plant → 4 joint_sim → 5 test_joint_sim。一次一步。

---

## 步骤 1. `sim/plant.hpp`

```cpp
// 关节空间植物：M ddq = tau - rnea(q, dq, 0)
#pragma once

#include "model/robot_model.hpp"
#include "types/joint.hpp"

namespace horkin
{
    /**
     * 无接触刚体积分器
     *
     * 说明
     * - 同一份 RobotModel，与真机 FK / 重力共用
     * - 不实现 IJointDriver；端口封装在 JointSim
     * - 一步半隐式欧拉，夹 URDF 限位
     */
    class Plant
    {
        public:
            /**
             * 参数
             * - model：已构造的刚体模型，必须比 Plant 活得长
             * - q0：初始位置，默认全 0
             */
            explicit Plant(const RobotModel& model, const JointVec& q0 = {});

            const JointVec& q() const;
            const JointVec& dq() const;

            /**
             * 直接改状态，给单测用
             *
             * 参数
             * - q / dq：新的位置、速度
             */
            void set_state(const JointVec& q, const JointVec& dq);

            /**
             * 积分一拍
             *
             * 流程：M(q) ddq = tau - rnea(q, dq, 0)，再半隐式欧拉，夹限位
             *
             * 参数
             * - tau：关节力（转动 N·m，移动 N）
             * - dt：周期，秒，必须 > 0
             */
            void step(const JointVec& tau, double dt);

        private:
            const RobotModel& model_;
            JointVec q_{};
            JointVec dq_{};
    };
}
```

## 步骤 2. `sim/plant.cpp`

```cpp
// Plant::step，关节空间半隐式欧拉
#include "sim/plant.hpp"

#include "types/inertia.hpp"
#include "types/joint_layout.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <stdexcept>

namespace horkin
{
    namespace
    {
        Eigen::MatrixXd to_matrix(const Inertia& M)
        {
            Eigen::MatrixXd out(static_cast<int>(kNJoints), static_cast<int>(kNJoints));
            for (std::size_t i = 0; i < kNJoints; ++i)
                for (std::size_t j = 0; j < kNJoints; ++j)
                    out(static_cast<int>(i), static_cast<int>(j)) = M[i][j];
            return out;
        }

        Eigen::VectorXd to_eigen(const JointVec& v)
        {
            Eigen::VectorXd out(static_cast<int>(kNJoints));
            for (std::size_t i = 0; i < kNJoints; ++i)
                out[static_cast<int>(i)] = v[i];
            return out;
        }

        JointVec from_eigen(const Eigen::VectorXd& v)
        {
            JointVec out{};
            for (std::size_t i = 0; i < kNJoints; ++i)
                out[i] = v[static_cast<int>(i)];
            return out;
        }
    }

    Plant::Plant(const RobotModel& model, const JointVec& q0)
        : model_(model), q_(q0)
    {
    }

    const JointVec& Plant::q() const { return q_; }
    const JointVec& Plant::dq() const { return dq_; }

    void Plant::set_state(const JointVec& q, const JointVec& dq)
    {
        q_ = q;
        dq_ = dq;
    }

    void Plant::step(const JointVec& tau, double dt)
    {
        if (!(dt > 0.0))
        {
            throw std::runtime_error("Plant::step dt must be > 0");
        }

        const Inertia M = model_.mass(q_);
        const JointVec nle = model_.rnea(q_, dq_, JointVec{});

        Eigen::VectorXd rhs = to_eigen(tau) - to_eigen(nle);
        const Eigen::VectorXd ddq = to_matrix(M).ldlt().solve(rhs);

        dq_ = from_eigen(to_eigen(dq_) + ddq * dt);
        q_ = from_eigen(to_eigen(q_) + to_eigen(dq_) * dt);

        const JointVec lower = model_.q_lower();
        const JointVec upper = model_.q_upper();
        for (std::size_t i = 0; i < kNJoints; ++i)
        {
            if (q_[i] < lower[i])
            {
                q_[i] = lower[i];
                dq_[i] = 0.0;
            }
            else if (q_[i] > upper[i])
            {
                q_[i] = upper[i];
                dq_[i] = 0.0;
            }
        }
    }
}
```

LDLT 能用，是因为 mass() 已经把 M 补成对称正定。dt 用新速度去积位置，这就是半隐式欧拉。

## 步骤 3. 把 `sim` 改成 STATIC

`sim/CMakeLists.txt` 整段换成：

```cmake
# 仿真后端。实现 ports；植物用同一份 kine/dyn。禁止链接 hal。

add_library(horkin_sim STATIC
  plant.cpp
)
target_link_libraries(horkin_sim
  PUBLIC horkin_ports horkin_dyn
  PRIVATE ${HORKIN_PINOCCHIO}
)
target_include_directories(horkin_sim PUBLIC "${HORKIN_INCLUDE_ROOT}")

if(HORKIN_BUILD_TESTS)
  add_executable(test_plant test/test_plant.cpp)
  target_link_libraries(test_plant PRIVATE horkin_sim)
  target_compile_definitions(test_plant PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")
endif()
```

PRIVATE ${HORKIN_PINOCCHIO} 只为了 Eigen 头和链接。cpp 里不要写 `#include <pinocchio/...>`。步骤 4 再把 `joint_sim.cpp` 加进源文件列表。现在不要加 contact / sensor_sim / world，文件还不存在。

`horkin_kine` 这一步不必 PUBLIC 链接：植物只用 mass / rnea。以后要显示末端再加。

## 步骤 4. `sim/test/test_plant.cpp`

两组：重力补偿应几乎静止；零力矩应明显运动。周期 1 ms，跑 0.5 s。姿态仍不要动 q[0]。

```cpp
#include "model/robot_model.hpp"
#include "sim/plant.hpp"
#include "types/joint.hpp"
#include "types/joint_layout.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>

#ifndef HORKIN_TEST_URDF
#define HORKIN_TEST_URDF "config/urdf/v6_mc.urdf"
#endif

namespace
{
    double max_abs(const horkin::JointVec& a)
    {
        double m = 0.0;
        for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        {
            const double v = std::abs(a[i]);
            if (v > m) m = v;
        }
        return m;
    }

    double max_abs_diff(const horkin::JointVec& a, const horkin::JointVec& b)
    {
        double m = 0.0;
        for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        {
            const double v = std::abs(a[i] - b[i]);
            if (v > m) m = v;
        }
        return m;
    }
}

int main()
{
    horkin::RobotModel robot(HORKIN_TEST_URDF, "tool");
    const double dt = 1e-3;
    const int nstep = 500;

    {
        horkin::Plant plant(robot);
        const horkin::JointVec q0 = plant.q();
        for (int k = 0; k < nstep; ++k)
        {
            plant.step(robot.gravity(plant.q()), dt);
        }
        if (max_abs_diff(plant.q(), q0) > 1e-3)
        {
            std::fprintf(stderr, "gravity hold drifted |dq|=%.4e\n",
                         max_abs_diff(plant.q(), q0));
            return 1;
        }
        if (max_abs(plant.dq()) > 1e-2)
        {
            std::fprintf(stderr, "gravity hold still moving |dq|=%.4e\n",
                         max_abs(plant.dq()));
            return 1;
        }
    }

    {
        horkin::Plant plant(robot);
        const horkin::JointVec q0 = plant.q();
        horkin::JointVec zero{};
        for (int k = 0; k < nstep; ++k)
        {
            plant.step(zero, dt);
        }
        if (max_abs_diff(plant.q(), q0) < 1e-4)
        {
            std::fprintf(stderr, "zero torque did not move the arm\n");
            return 1;
        }
    }

    std::printf("test_plant ok\n");
    return 0;
}
```

编译只编植物测试：

```bash
source scripts/env.sh
cmake -S . -B build -DHORKIN_BUILD_TESTS=ON
cmake --build build --target test_plant
./build/sim/test_plant
```

选项是 HORKIN_BUILD_TESTS（有 S）。日常不必带上 test_fk。

## 步骤 5. `sim/joint_sim.hpp`

Plant 测过后再包端口。

```cpp
// 仿真关节驱动，实现 IJointDriver
#pragma once

#include "ports/joint_driver.hpp"
#include "sim/plant.hpp"
#include "types/command.hpp"
#include "types/joint.hpp"

namespace horkin
{
    /**
     * 仿真后端：Write 收 tau_ff，内部积分植物，Read 回 q、dq
     *
     * 说明
     * - S0 忽略 cmd.q / dq / kp / kd
     * - Enable 之前 Write 不积分，Read 的 valid=false
     */
    class JointSim : public IJointDriver
    {
        public:
            /**
             * 参数
             * - model：刚体模型，必须比 JointSim 活得长
             * - dt：积分周期，秒
             * - q0：初始关节
             */
            explicit JointSim(const RobotModel& model, double dt,
                              const JointVec& q0 = {});

            void Enable() override;
            void Disable() override;
            void Read(JointState& out) override;
            void Write(const JointCommand& cmd) override;

        private:
            Plant plant_;
            double dt_;
            bool enabled_{false};
            JointVec last_tau_{};
    };
}
```

## 步骤 6. `sim/joint_sim.cpp`

```cpp
#include "sim/joint_sim.hpp"

namespace horkin
{
    JointSim::JointSim(const RobotModel& model, double dt, const JointVec& q0)
        : plant_(model, q0), dt_(dt)
    {
        if (!(dt_ > 0.0))
        {
            throw std::runtime_error("JointSim dt must be > 0");
        }
    }

    void JointSim::Enable() { enabled_ = true; }
    void JointSim::Disable() { enabled_ = false; }

    void JointSim::Read(JointState& out)
    {
        out.q = plant_.q();
        out.dq = plant_.dq();
        out.tau = last_tau_;
        out.valid = enabled_;
    }

    void JointSim::Write(const JointCommand& cmd)
    {
        if (!enabled_)
        {
            return;
        }
        last_tau_ = cmd.tau_ff;
        plant_.step(cmd.tau_ff, dt_);
    }
}
```

`joint_sim.cpp` 需要 `#include <stdexcept>`（若把 dt 检查放在 cpp）。CMake 源文件列表加上 `joint_sim.cpp`。

## 步骤 7. `sim/test/test_joint_sim.cpp`

确认走的是端口，不是直接碰 Plant：Enable → 循环 Write(tau_ff=gravity) → Read 仍近似静止。

```cpp
#include "model/robot_model.hpp"
#include "sim/joint_sim.hpp"
#include "types/command.hpp"
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
    const double dt = 1e-3;
    horkin::JointSim joints(robot, dt);

    horkin::JointState st{};
    joints.Read(st);
    if (st.valid)
    {
        std::fprintf(stderr, "Read valid before Enable\n");
        return 1;
    }

    joints.Enable();
    joints.Read(st);
    const horkin::JointVec q0 = st.q;

    horkin::JointCommand cmd{};
    for (int k = 0; k < 500; ++k)
    {
        joints.Read(st);
        cmd.tau_ff = robot.gravity(st.q);
        joints.Write(cmd);
    }
    joints.Read(st);
    if (!st.valid)
    {
        std::fprintf(stderr, "Read invalid after Enable\n");
        return 1;
    }

    double dp = 0.0;
    for (std::size_t i = 0; i < horkin::kNJoints; ++i)
        dp += std::abs(st.q[i] - q0[i]);
    if (dp > 1e-3)
    {
        std::fprintf(stderr, "JointSim gravity hold drifted dp=%.4e\n", dp);
        return 1;
    }

    std::printf("test_joint_sim ok\n");
    return 0;
}
```

CMake 的 TESTS 段再加一套 `test_joint_sim`，照抄 test_plant 那四行，目标名换掉。不要改已经能过的 test_plant。

```bash
cmake --build build --target test_joint_sim
./build/sim/test_joint_sim
```

## 约定

- 文件名小写加下划线。注释中文，风格跟 model/robot_model.hpp、dyn/mass.cpp 一样。
- sim 禁止链接 hal。Plant / JointSim 禁止包含 pinocchio 头和 model/detail/impl.hpp。
- 结果都是 JointVec；Eigen 矩阵出不了 plant.cpp。
- 不要做 mass 的重复实现，不要在 sim 里手写 tau = m*g*cos(q)。
- Pinocchio 只用 2.7。编译前 source scripts/env.sh。

S0 过了之后才是：servo_sim（用 kp/kd 把 JointCommand 变成力矩）→ 接触弹簧 → 假六维力。那些不要提前写。
