# 换设备续写 JointSim

Plant 已通过 `test_plant`。`sim/joint_sim.hpp` 已写好。接下来把植物包成 `IJointDriver`。

离开前把本文件和未提交代码一起提交并推送。新设备打开仓库后，把下面「继续 Horkin Core」那一整段贴进新的 Cursor 对话。一次只做一个步骤。

---

继续 Horkin Core 的仿真 S0，仓库是 horkin-core。Plant 已通过 test_plant。接下来做 JointSim（把植物包成 IJointDriver）。我要自己手敲。你先读现有文件。每次只告诉我下一步该写哪一个文件；我贴出代码后你再对照、纠错，并在我同意后帮我编译跑测试。不要重写 plant / kine / dyn / model，不要做 contact、sensor_sim、servo_sim、friction。

已完成，不要推倒：

- RobotModel：fk、jacobian、rnea、gravity、mass。test_fk / test_ik / test_rnea / test_mass 已过。
- URDF：config/urdf/v6_mc.urdf，末端帧 tool。kNJoints=6，下标 2 是移动副（米）。
- sim/plant.hpp/.cpp：半隐式欧拉，M ddq = tau - rnea(q,dq,0)，夹限位。test_plant ok。
- sim/CMakeLists.txt 已是 STATIC，目前只编 plant.cpp，已有 test_plant。
- sim/joint_sim.hpp 已写好（声明 JointSim : IJointDriver，成员 plant_、dt_、enabled_、last_tau_）。不要重写这个头。
- to_eigen 必须返回 Eigen::VectorXd。MatrixXd(n) 是 n×n 方阵。plant.cpp 里已经改成 VectorXd。

步骤顺序：1 joint_sim.cpp → 2 CMake 加上 joint_sim.cpp 和 test_joint_sim → 3 test_joint_sim.cpp。一次一步。S0 的 Write 只取 cmd.tau_ff，忽略 q/dq/kp/kd。Enable 之前 Write 不积分，Read 的 valid=false。

## 步骤 1. `sim/joint_sim.cpp`

```cpp
#include "sim/joint_sim.hpp"

#include <stdexcept>

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

`:` 后面用传入的 model、q0 构造成员 plant_，dt_ 存积分周期。函数体只检查 dt>0。Write 未使能则直接 return，不调用 plant.step。

## 步骤 2. `sim/CMakeLists.txt`

源文件列表加上 joint_sim.cpp。TESTS 段保留 test_plant，再抄一套 test_joint_sim，不要改已经能过的 test_plant。

```cmake
add_library(horkin_sim STATIC
  plant.cpp
  joint_sim.cpp
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

  add_executable(test_joint_sim test/test_joint_sim.cpp)
  target_link_libraries(test_joint_sim PRIVATE horkin_sim)
  target_compile_definitions(test_joint_sim PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")
endif()
```

先不要 cmake --build，下一步才写测试文件。

## 步骤 3. `sim/test/test_joint_sim.cpp`

走端口，不直接碰 Plant。Enable 前 Read 的 valid 必须是 false；Enable 后循环 Write(tau_ff=gravity)，0.5 s 后姿态几乎不变。

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

注释只标「Enable 前 valid 应为 false」「重力托住应几乎静止」，风格跟 test_plant 一样。

```bash
source scripts/env.sh
cmake -S . -B build -DHORKIN_BUILD_TESTS=ON
cmake --build build --target test_joint_sim
./build/sim/test_joint_sim
```

选项是 HORKIN_BUILD_TESTS（有 S）。日常不必编 test_fk。

约定：文件名小写加下划线；注释中文；sim 禁止链接 hal、禁止包含 pinocchio 头和 model/detail/impl.hpp。S0 过了之后才是 servo_sim → 接触弹簧 → 假六维力，那些不要提前写。
