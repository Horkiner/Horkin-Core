# 换设备续写动力学

离开前把本文件和未提交的代码一起提交并推送。另一台机器需要看得到 `model/robot_model.hpp` 里的 `rnea` / `gravity` 声明，以及 `dyn/rnea.cpp`。

新设备打开本仓库后，把下面整段贴进新的 Cursor 对话。

---

继续 Horkin Core 的动力学，仓库是 horkin-core。我要自己手敲剩余代码。你先读现有文件，缺什么就告诉我该写哪一段；我贴出代码后你再对照、纠错，并在我同意后帮我编译跑测试。不要重写已经写好的 rnea，不要改运动学，不要做 mass / friction / Inertia。

已完成，不要推倒：

- RobotModel 在 model/。公开头 model/robot_model.hpp 不暴露 pinocchio。
- Pinocchio 的 Model 和 Data 在 model/detail/impl.hpp。只有 model/、kine/、dyn/ 的 cpp 能包含它。
- 转换函数已有：detail::to_qpin、detail::to_vpin、detail::from_tau。
- 运动学：kine/fk.cpp、kine/jacobian.cpp、kine/ik_dls.cpp。horkin_kine 链接 horkin_model。
- 构造函数读 config/urdf/v6_mc.urdf，末端帧名 tool。kNJoints = 6，第 3 轴（下标 2）是移动副，单位米；其余是转动副，单位弧度。
- model/robot_model.hpp 已声明：
  - JointVec rnea(const JointVec& q, const JointVec& dq, const JointVec& ddq) const;
  - JointVec gravity(const JointVec& q) const;
- dyn/rnea.cpp 已实现：pinocchio::rnea(impl_->model, impl_->data, qpin, vpin, apin)，再用 detail::from_tau 按值返回。这段保留。

还没做：

1. dyn/gravity.cpp
   实现 RobotModel::gravity。包含 model/detail/impl.hpp 和 pinocchio/algorithm/rnea.hpp。
   调用 pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, qpin)，再 detail::from_tau。
   不要写成 return rnea(q, 零, 零)。Pinocchio 2.7 里 computeGeneralizedGravity 等价于 rnea(q,0,0)，但单测要拿它和 rnea 互相核对，调用自己就会变成同函数比较。
   静止前馈是 tau_ff = robot.gravity(q)。转动关节 N·m，移动关节 N。
2. dyn/CMakeLists.txt
   现在还是 INTERFACE，所以 rnea.cpp 根本没参加编译。改成 STATIC，源文件只列 rnea.cpp 和 gravity.cpp。
   PUBLIC 链接 horkin_model，PRIVATE 链接 ${HORKIN_PINOCCHIO}，PUBLIC include 用 ${HORKIN_INCLUDE_ROOT}。
   不要加入 mass.cpp、friction.cpp，文件还不存在。
3. dyn/test/test_rnea.cpp，在 HORKIN_BUILD_TESTS 下加入 test_rnea，链接 horkin_dyn，编译定义
   HORKIN_TEST_URDF=${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf
   三组断言：
   - 任意 q（含全 0，以及 q[0]=0.4）上，gravity(q) 与 rnea(q, 0, 0) 逐分量差小于 1e-9。
   - q[0]=0.4 的重力与全 0 姿态的重力不同（阈值 1e-6）。q[0] 是转动关节。
   - 同一 q 上 dq[0]=2（rad/s），rnea(q, dq, 0) 与 gravity(q) 不同（阈值 1e-6），用来确认速度项真的算了。

约定：

- 文件名继续小写加下划线。注释用中文，风格跟 model/robot_model.hpp、kine/fk.cpp 一样。
- 成员函数是 const。impl_->data 是共用工作区，结果必须按值拷出。
- 惯性矩阵以后才做。到时在 types/ 加 Inertia（n×n 的 std::array），接口是 Inertia mass(q)。非线性项 c(q,dq) 仍用 JointVec，不要新类型。这次不要提前加 Inertia。
- 依赖：kine 和 dyn 都只依赖 model，dyn 不依赖 kine。
- Pinocchio 只用 2.7。编译前 source scripts/env.sh。

验证：

```bash
source scripts/env.sh
cmake -S . -B build -DHORKIN_BUILD_TESTS=ON
cmake --build build --target test_rnea test_fk test_ik
./build/dyn/test_rnea
./build/kine/test_fk
./build/kine/test_ik
```

运动学测试也要过，确认这次改动没有把 fk / ik 编坏。
