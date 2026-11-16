# 06 · 知识体系、学习路径与项目实施路线

> 目标：把“做 WBC”需要的知识拆成可学习的模块，给出顺序、资料和练习；
> 同时给出你这个项目的分阶段实施路线和验收标准。

---

## 1. 知识地图

```text
                        ┌──────────────────────────────┐
                        │ 数学基础                      │
                        │ 线代 / 李群 / 优化 / 数值      │
                        └──────────────┬───────────────┘
                                       │
        ┌──────────────────────────────┼──────────────────────────────┐
        │                              │                              │
┌───────▼────────┐          ┌──────────▼──────────┐        ┌──────────▼─────────┐
│ 刚体力学        │          │ 控制理论             │        │ 机器人软件          │
│ 运动学/动力学   │          │ PD/计算力矩/OSC/MPC  │        │ ROS 2 / Pinocchio  │
│ 空间代数        │          │ 阻抗/导纳/分层       │        │ Isaac / cuRobo     │
│ 浮动基/接触     │          │ 稳定性/频域          │        │ QP 求解器/实时      │
└───────┬────────┘          └──────────┬──────────┘        └──────────┬─────────┘
        │                              │                              │
        └──────────────────────────────┼──────────────────────────────┘
                                       │
                        ┌──────────────▼───────────────┐
                        │ WBC                          │
                        │ 任务建模 / 约束 / QP / 分级   │
                        └──────────────┬───────────────┘
                                       │
                        ┌──────────────▼───────────────┐
                        │ 硬件与工程                    │
                        │ 驱动器/传感器/标定/安全/延迟  │
                        └──────────────────────────────┘
```

### 1.1 数学

| 模块 | 需要掌握 | 在 WBC 里的用途 |
|---|---|---|
| 线性代数 | 矩阵分解、伪逆、零空间、最小二乘、条件数 | 雅可比、QP、零空间投影 |
| 李群/李代数 | SO(3)/SE(3)、exp/log、左右雅可比、四元数 | 姿态误差、积分、浮动基 |
| 凸优化 | QP、KKT、对偶、有效集、内点法 | WBC 的求解 |
| 数值方法 | 阻尼最小二乘、正则化、SVD、稳定性 | 奇异位形、数值鲁棒性 |
| 概率/估计（选） | 卡尔曼滤波、状态估计 | 真机状态估计 |

### 1.2 刚体力学

| 模块 | 内容 |
|---|---|
| 运动学 | 正/逆运动学、雅可比、frame、参考系 |
| 空间代数 | Motion/Force、SE(3) 作用、伴随变换 |
| 动力学 | 牛顿-欧拉、拉格朗日、RNEA、CRBA、ABA |
| 浮动基 | 广义坐标、`nq≠nv`、基座动力学 |
| 接触 | 接触约束、摩擦锥、Baumgarte、接触切换 |
| 质心 | CoM、centroidal map、动量 |

### 1.3 控制

| 模块 | 内容 |
|---|---|
| 关节控制 | PD、前馈、重力补偿、摩擦补偿 |
| 计算力矩 | `tau = M(qdd* + Kp e + Kd ė) + h` |
| 任务空间 | OSC、阻抗、导纳 |
| 多任务 | 加权、零空间、HQP |
| 预测控制 | MPC、DDP、直接法、实时迭代 |
| 稳定性 | Lyapunov、无源性、延迟裕度、饱和 |

### 1.4 软件

| 模块 | 内容 |
|---|---|
| Pinocchio | 模型、FK、J、M/h、CoM、李群 |
| ROS 2 | 话题、action、`ros2_control`、`use_sim_time` |
| Isaac Sim | articulation、joint drive、ROS 2 bridge |
| cuRobo | 全身规划、碰撞球、轨迹输出 |
| QP 求解器 | OSQP、ProxSuite、quadprog、Clarabel |
| 实时 | 零分配、线程、优先级、延迟测量 |

### 1.5 硬件（真机）

| 模块 | 内容 |
|---|---|
| 驱动器 | 位置/速度/力矩模式、增益、限幅、使能 |
| 传感器 | 编码器、IMU、力/力矩传感器 |
| 标定 | 零位、符号、减速比、armature、摩擦 |
| 安全 | 看门狗、急停、限位、碰撞检测 |
| 通信 | EtherCAT/CAN/ROS 2、周期、抖动 |

---

## 2. 学习路径（建议 6–8 周，边学边做）

### 阶段 A：数学与运动学（1–2 周）

- 线性代数复习：伪逆、零空间、最小二乘、SVD。
- 李群入门：SO(3)/SE(3)、exp/log、四元数。
- 资料：
  - Sola et al., *A micro Lie theory for state estimation in robotics*（arXiv:1812.01537）。
  - Lynch & Park, *Modern Robotics*，第 3 章（刚体运动）。
- 练习：
  1. 用 Pinocchio 对随机 `q` 算 `oMf`，用 `log3/exp3` 验证 `R = exp3(log3(R))`。
  2. 验证 `difference/integrate` 的逆关系。

### 阶段 B：刚体动力学（2–3 周）

- 牛顿-欧拉、RNEA、CRBA、ABA 的思想。
- 空间代数：Motion/Force、SE(3) 作用。
- 浮动基：`nq≠nv`、基座动力学。
- 资料：
  - Featherstone, *Rigid Body Dynamics Algorithms*（前 6 章）。
  - Pinocchio 文档的 “Feature” 部分（spatial、model/data、joints、dynamics）。
  - 本仓库 `doc/a-features/`。
- 练习：
  1. 用 `crba` 和 `rnea` 验证 `tau = M qdd + h`。
  2. 用 `aba` 验证 `qdd = M^{-1}(tau - h)`。
  3. 对 2-DoF 平面臂手推 `M`，和 Pinocchio 对比。

### 阶段 C：任务空间控制与加权 QP（2 周）

- 任务雅可比、误差、期望加速度。
- 加权 QP、约束、求解器。
- 资料：
  - Khatib 1987（操作空间）。
  - Siciliano & Khatib 手册中任务空间控制章节。
  - Boyd, *Convex Optimization*（第 4、5、11 章）。
  - OSQP 文档。
- 练习：
  1. 实现单末端 6D 任务，验证 `J qdd = a* - Jdot qd`。
  2. 加 posture 任务，调权重，观察冗余。
  3. 加力矩/速度/位置约束，检查可行性。

### 阶段 D：分级控制（1–2 周）

- 零空间、严格优先级、HQP。
- 资料：
  - Siciliano & Slotine 1991。
  - Sentis & Khatib 2005。
  - Kanoun et al. 2011。
  - Escande et al. 2014。
  - Herzog et al. 2016。
- 练习：
  1. 实现两级 HQP（末端 > posture）。
  2. 制造任务冲突，比较加权 QP 和 HQP。
  3. 用 TSID 跑一个例子，读它的任务/接触配置。

### 阶段 E：浮动基、接触、质心（2–4 周）

- 浮动基动力学、接触约束、摩擦锥、centroidal。
- 资料：
  - Featherstone 第 7–9 章。
  - Pinocchio 的 centroidal/constrained dynamics 文档。
  - Crocoddyl 论文与文档。
  - Bellicoso et al. 2018（ANYmal WBC）。
- 练习：
  1. 浮动基模型，验证 `Ag @ v = hg`。
  2. 加接触约束 `Jc qdd + Jdotc qd = 0`，验证接触点不动。
  3. 简单的质心平衡 QP。

### 阶段 F：真机与工程（持续）

- 系统辨识、摩擦、延迟、安全、日志。
- 资料：驱动器手册、机器人学实验手册、团队经验。
- 练习：悬吊试跑、低增益、小幅度、逐步放开。

---

## 3. 资料清单

### 3.1 书

| 书 | 用途 |
|---|---|
| Featherstone, *Rigid Body Dynamics Algorithms* | 动力学算法圣经 |
| Lynch & Park, *Modern Robotics* | 运动学/动力学入门，免费预印本 |
| Siciliano & Khatib, *Springer Handbook of Robotics* | 机器人学百科 |
| Boyd & Vandenberghe, *Convex Optimization* | QP/优化基础，免费 PDF |
| Barfoot, *State Estimation for Robotics* | 李群/估计 |
| Tedrake, *Underactuated Robotics* | 控制/优化视角，在线免费 |
| Craig, *Introduction to Robotics* | 经典教材 |

### 3.2 论文

| 论文 | 主题 |
|---|---|
| Khatib 1987, *A unified approach for motion and force control of robot manipulators: The operational space formulation* | 操作空间控制 |
| Siciliano & Slotine 1991, *A general framework for managing multiple tasks in highly redundant robotic systems* | 任务优先级 |
| Sentis & Khatib 2005, *Synthesis of whole-body behaviors through hierarchical control of behavioral primitives* | 全身分层控制 |
| Kanoun, Lamiraux, Wieber 2011, *Prioritizing linear equality and inequality systems* | 带不等式的优先级 |
| Escande, Mansard, Wieber 2014, *Hierarchical quadratic programming* | HQP |
| Herzog et al. 2016, *Balancing experiments on a torque-controlled humanoid with hierarchical inverse dynamics* | 人形 HQP 实践 |
| Bellicoso et al. 2018, *Dynamic locomotion and whole-body control for quadrupedal robots* | 四足 WBC |
| Koenemann et al. 2015, *Whole-body model-predictive control applied to the HRP-2 humanoid* | 全身 MPC |
| Carpentier et al. 2019, *The Pinocchio C++ library* | Pinocchio |
| Mastalli et al. 2020, *Crocoddyl* | 多接触最优控制 |
| Sola et al. 2018, *A micro Lie theory for state estimation in robotics* | 李群入门 |

### 3.3 课程

- MIT 6.832 *Underactuated Robotics*（Tedrake，在线）。
- ETH *Robot Dynamics*（Hutter）。
- Coursera *Modern Robotics*（Lynch）。
- Stanford CS223A *Introduction to Robotics*。

### 3.4 文档/代码

- Pinocchio：<https://gepetto.github.io/doc/pinocchio/doxygen-html/>
- Pinocchio cheat sheet：仓库 `doc/pinocchio_cheat_sheet.pdf`
- TSID：<https://stack-of-tasks.github.io/tsid/>
- Crocoddyl：<https://crocoddyl.readthedocs.io/>
- Aligator：<https://simple-robotics.github.io/aligator/>
- Pink：<https://github.com/stephane-caron/pink>
- OSQP：<https://osqp.org/>
- ProxSuite：<https://github.com/Simple-Robotics/proxsuite>
- Isaac Sim ROS 2 bridge：NVIDIA 文档
- cuRobo：NVIDIA 文档

---

## 4. 项目路线图（可执行）

### P0 · 模型对齐（1–2 天）

- 同一 URDF 建 Pinocchio 固定基座模型、Isaac articulation、cuRobo 配置。
- 关节名/顺序/零位/符号/单位核对。
- 随机 10 组 `q`，比较 Pinocchio FK 与 Isaac FK。
- 验收：末端位置误差 < 1e-6 m；关节数一致。

### P1 · 轨迹直通（2–3 天）

- cuRobo → `JointTrajectory` → bridge → PD。
- 静态目标、慢速轨迹。
- 验收：无超限、无抖动、跟踪误差可接受。

### P2 · WBC 骨架（3–5 天）

- WBC 节点订阅 `/joint_states` 和轨迹；发布 `q_des/qd_des`。
- 先只做 posture 跟踪（`qdd` 跟踪 `q_ref`）。
- 验收：与 P1 跟踪效果相当，QP 200 Hz 稳定。

### P3 · 末端 + 躯干任务（1 周）

- 左右末端 6D + 躯干 3D + posture；加权 QP。
- 验收：末端笛卡尔误差优于 P1；无奇异/发散。

### P4 · 约束（3–5 天）

- 力矩/速度/位置/加速度限位；QP 失败处理。
- 验收：不超限；QP 失败率 < 1%；失败时安全保持。

### P5 · 前馈（3–5 天）

- `tau_g` 前馈；再考虑 `M qdd + h`。
- 验收：稳态误差下降；无振荡。

### P6 · 真机（2–4 周）

- 替换桥接、标定、安全层、低增益试跑。
- 验收：悬吊稳定；低速轨迹；逐步全速。

### P7 · 底盘（以后）

- 浮动基/平面基座、轮地接触、centroidal。
- 验收：平衡/移动时 WBC 稳定。

---

## 5. 练习清单（从易到难）

1. Pinocchio FK 与 Isaac FK 对比。
2. `tau = M qdd + h` 与 `rnea` 对比。
3. 单关节 PD 跟踪正弦。
4. 单末端 6D 任务，验证 `J qdd = a* - Jdot qd`。
5. 加 posture，观察零空间。
6. 加权 QP，调权重，画任务误差。
7. 加力矩约束，观察饱和。
8. 两级 HQP，比较加权 QP。
9. 浮动基模型，验证 `hg = Ag v`。
10. 接触约束，验证接触点不滑动。
11. 质心平衡 QP。
12. 用 TSID 复现一个任务栈。
13. 用 Crocoddyl 跑一个简单 MPC。
14. 真机低增益试跑。

---

## 6. 如何用 Pinocchio 仓库学习

- 读 `doc/a-features/`：spatial、model/data、joints、dynamics、derivatives。
- 跑 `examples/`：`overview-simple`、`inverse-dynamics`、`kinematics-derivatives`、`simulation-*`。
- 读 `unittest/`：每个算法都有对应测试，是最好的用法示例。
- 读 `include/pinocchio/algorithm/*.hpp`：公共 API 和文档注释。
- 读 `include/pinocchio/src/algorithm/*.hxx`：实现（visitor 模式）。
- 用 `benchmark/` 了解性能。

---

## 7. 一页总结

```text
知识体系 = 数学（线代/李群/优化）
         + 力学（运动学/动力学/接触）
         + 控制（PD/任务空间/分层/MPC）
         + 软件（Pinocchio/ROS 2/Isaac/cuRobo/QP）
         + 硬件（驱动/标定/安全）

学习顺序 = 数学 → 动力学 → 任务空间 QP → 分层 → 浮动基/接触 → 真机。

项目路线 = P0 模型对齐 → P1 轨迹直通 → P2 WBC 骨架
         → P3 任务 → P4 约束 → P5 前馈 → P6 真机 → P7 底盘。

先跑通，再优化；先加权，再分层；先固定基座，再浮动基。
```

回到 [README.md](README.md) 查看文档索引。
