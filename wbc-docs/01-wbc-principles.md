# 01 · WBC 的具体原理

> 目标：把“全身控制”从一句口号拆成可计算的量：**模型 → 任务 → 约束 → QP → 关节命令**。
> 读完后你应该能回答：WBC 在优化什么？为什么是加速度级？固定基座时它简化成什么？

---

## 1. WBC 是什么

**Whole-Body Control（全身控制）**：对一个高自由度机器人，同时控制多个任务（末端、躯干、质心、姿态……），并在满足动力学、接触、关节限位等约束的前提下，输出关节力矩或关节目标。

它和“关节空间轨迹跟踪”的区别：

| | 关节轨迹跟踪 | WBC |
|---|---|---|
| 输入 | 每个关节的 q(t) | 任务（笛卡尔位姿、CoM、姿态）+ 约束 |
| 冗余利用 | 不利用，直接播 | 用零空间/权重分配冗余 |
| 约束处理 | 事后 clip | 在优化里显式处理 |
| 冲突处理 | 无 | 加权或严格优先级 |
| 输出 | q_des | qdd / tau / q_des |

WBC 不是某一个算法，而是一类**“任务 + 约束 → 优化”**的框架。具体用什么优化形式（加权 QP、分层 QP、操作空间法、MPC）是路线选择，见 [03-wbc-routes.md](03-wbc-routes.md)。

---

## 2. 记号与机器人模型

### 2.1 广义坐标与速度

- 固定基座：`q ∈ R^n`，`v = qd ∈ R^n`，`a = qdd ∈ R^n`。你的机器人 `n = 18`（4 躯干 + 14 手臂）。
- 浮动基座：`q = [q_b; q_j]`，`q_b ∈ SE(3)` 用 7 维表示（位置 + 四元数），`v = [v_b; v_j] ∈ R^6 × R^{nj}`。此时 `nq = 7 + nj`，`nv = 6 + nj`，**nq ≠ nv**。
- 关节速度/加速度都在切空间 `R^nv` 里；位置更新要用李群积分 `q_next = integrate(q, v·dt)`，误差要用 `difference(q1, q2)`，不能直接相减（浮动基/四元数会错）。

### 2.2 空间代数

Pinocchio 用 6 维空间向量表示运动与力：

- `Motion` = `[linear; angular]`（线速度在前，角速度在后）。
- `Force` = `[force; torque]`。
- `SE3` 表示刚体变换；`Motion`/`Force` 的坐标依赖参考系。
- 常用参考系：`LOCAL`（随体坐标系）、`WORLD`（世界原点）、`LOCAL_WORLD_ALIGNED`（原点在物体上、轴向对齐世界）。

WBC 里最重要的约定：**雅可比、速度、误差、力必须在同一参考系下配对**。混用是绝大多数“符号/量纲不对”的来源。

### 2.3 刚体动力学

固定基座：

```text
M(q) qdd + C(q, qd) qd + g(q) = S^T tau
```

浮动基座 + 接触：

```text
M(q) qdd + h(q, qd) = S^T tau + sum_i J_i(q)^T f_i
h(q, qd) = C(q, qd) qd + g(q)
```

其中：

- `M`：关节空间质量矩阵（`nv × nv`，对称正定）。
- `h`：非线性项（科氏/离心 + 重力），Pinocchio 里是 `data.nle`。
- `S`：驱动选择矩阵。浮动基下 `S = [0_{nj×6}  I_{nj}]`，即 `S^T tau` 只作用在驱动关节。
- `J_i`：第 i 个接触点的 6D 雅可比；`f_i`：接触力/力矩。
- `tau`：关节力矩（`nj` 维，浮动基下不含基座 6 维）。

固定基座时没有 `S`，`tau` 直接是 `nv` 维。

### 2.4 固定基座 vs 浮动基座

| | 固定基座 | 浮动基座 |
|---|---|---|
| 状态 | `nq = nv` | `nq ≠ nv`（四元数） |
| 动力学 | `M qdd + h = tau` | `M qdd + h = S^T tau + J_c^T f` |
| 接触 | 不需要（基座固定） | 必须处理（脚/轮/手） |
| WBC 变量 | 通常只有 `qdd` | `qdd` + `f`（+ `tau`） |
| 平衡 | 不需要 | 需要 CoM/centroidal + 摩擦锥 |
| 你的当前阶段 | ✅ | 以后解锁轮式底盘再做 |

**你的场景是左列**：固定基座、无接触，WBC 不需要接触力分配，规模小很多。

---

## 3. 任务（Task）

### 3.1 任务的一般形式

一个任务定义了一个希望控制的输出 `x ∈ R^m`（m 通常 3 或 6），它是 `q` 的函数：

```text
x = f(q)
xd = J(q) qd
xdd = J(q) qdd + Jdot(q, qd) qd
```

- `J = ∂f/∂q`：任务雅可比（`m × nv`）。
- `Jdot qd`：任务空间偏置项（科氏/离心），高速时不能省。
- 任务的目标是让 `x` 跟踪 `x_des(t)`。

### 3.2 误差定义

**位置任务**（3D）：

```text
e_p = p_des - p
```

**姿态任务**（3D）：

```text
e_R = log3(R_des R^T)      # 世界系旋转误差
# 或
e_R = log3(R^T R_des)      # 局部系旋转误差
```

两者都合法，但必须和雅可比的参考系一致：

- 用 `LOCAL_WORLD_ALIGNED` 雅可比 → 角速度是世界轴 → 用 `log3(R_des R^T)`。
- 用 `LOCAL` 雅可比 → 角速度是局部轴 → 用 `log3(R^T R_des)`。

**6D 位姿任务**：

```text
e = [e_p; e_R] ∈ R^6
```

### 3.3 任务空间 PD

给定参考 `x_des, xd_des, xdd_des`，期望加速度取：

```text
xdd* = xdd_des + Kp e + Kd (xd_des - xd)
```

- `Kp`、`Kd` 是任务空间增益，通常取临界阻尼 `Kd = 2 sqrt(Kp)`（按单位调整）。
- 位置和姿态可以用不同的增益（姿态通常更软）。
- 这就是“任务空间 PD + 前馈加速度”。

于是任务变成关于 `qdd` 的线性方程：

```text
J(q) qdd = xdd* - Jdot(q, qd) qd
```

**软任务**把它放进代价函数，**硬任务**把它作为等式约束。

### 3.4 常见任务

| 任务 | 维度 | 典型用途 | Pinocchio 量 |
|---|---|---|---|
| 末端位姿 | 6 | 双手抓取/操作 | `data.oMf[frame_id]`、`getFrameJacobian` |
| 末端位置 | 3 | 只关心位置 | 取 6D 雅可比的前 3 行 |
| 躯干姿态 | 3 | 保持躯干竖直/前倾 | 躯干 frame 的旋转部分 |
| 关节 posture | n | 冗余分解、保持舒适姿态 | 单位阵 |
| CoM | 3 | 重心位置（浮动基平衡） | `centerOfMass`、`jacobianCenterOfMass` |
| 质心动量 | 6 | 浮动基平衡/抗扰 | `data.Ag`、`data.hg` |
| 接触 | 6/3 | 脚/手不滑动 | 接触点 frame 雅可比 |

对你的固定基座机器人，最常用的是：**左右末端 6D + 躯干 3D + posture**。

---

## 4. 约束（Constraint）

### 4.1 关节限位

- 位置：`q_min ≤ q ≤ q_max`。在加速度级，严格处理是非线性的；工程上常用两种近似：
  - 用 `q + qd·dt + 0.5·qdd·dt²` 的上下界，转成 `qdd` 的线性不等式；
  - 或者求解后对 `q_des` 做 clip（简单，但可能破坏任务）。
- 速度：`qd_min ≤ qd + qdd·dt ≤ qd_max`，对 `qdd` 是线性的。
- 加速度：`qdd_min ≤ qdd ≤ qdd_max`，直接 box 约束。

### 4.2 力矩限位

固定基座：

```text
tau = M(q) qdd + h(q, qd)
tau_min ≤ tau ≤ tau_max
```

代入后是 `qdd` 的线性不等式：

```text
tau_min - h ≤ M qdd ≤ tau_max - h
```

浮动基座时 `tau` 只取驱动关节对应的行（`S` 的选择）。

### 4.3 接触约束（浮动基）

接触点不滑动：

```text
J_c(q) qdd + Jdot_c(q, qd) qd = 0
```

工程上加 Baumgarte 稳定项，抑制数值漂移：

```text
J_c qdd + Jdot_c qd = -2 ζ ω (J_c qd) - ω² e_c
```

其中 `e_c` 是接触点的位姿误差。`ζ≈0.7–1.0`，`ω` 按控制频率选。

### 4.4 摩擦锥

接触力在接触系下：

```text
f_z ≥ 0
sqrt(f_x² + f_y²) ≤ μ f_z          # 真实圆锥（SOCP）
|f_x| ≤ μ f_z, |f_y| ≤ μ f_z       # 四棱锥线性化（QP）
```

四棱锥偏保守，但让问题保持 QP；精确圆锥需要 SOCP 求解器（Clarabel/ECOS/HPIPM）。

### 4.5 碰撞/自碰撞

- 规划层（cuRobo）已经保证无碰撞；
- WBC 的局部修正可能破坏无碰撞性；
- 需要时加线性化的碰撞约束（`J_col qdd ≥ -ξ`），或用 Pinocchio + coal 做距离查询；
- 固定基座、慢速时，通常限制“WBC 修正量不要偏离规划太远”就够了。

---

## 5. WBC 的 QP

### 5.1 一般形式（浮动基 + 接触）

决策变量：`qdd`、`f`（可选 `tau`）。

```text
min_{qdd, f, tau}   Σ_i w_i || J_i qdd + Jdot_i qd - xdd_i* ||²
                  + w_tau ||tau||²
                  + w_f   ||f||²

s.t.  M qdd + h = S^T tau + Σ_i J_i^T f_i        (动力学)
      J_c qdd + Jdot_c qd = 0                    (接触不滑动)
      f_z ≥ 0, |f_x| ≤ μ f_z, |f_y| ≤ μ f_z      (摩擦锥)
      tau_min ≤ tau ≤ tau_max                    (力矩限位)
      qdd_min ≤ qdd ≤ qdd_max                    (加速度/速度限位)
```

这是一个**凸 QP**（四棱锥线性化后），可以用 OSQP/ProxSuite/quadprog 求解。变量规模：`nv + 6*nc + nj`，人形通常几十到一百多维。

### 5.2 固定基座简化（你的场景）

没有接触力 `f`，没有 `S`，动力学就是 `tau = M qdd + h`。于是 QP 只对 `qdd` 优化：

```text
min_{qdd}   Σ_i w_i || J_i qdd + Jdot_i qd - xdd_i* ||²
          + w_r ||qdd - qdd_ref||²

s.t.  qdd_min ≤ qdd ≤ qdd_max
      tau_min - h ≤ M qdd ≤ tau_max - h
      qd_min - qd ≤ qdd·dt ≤ qd_max - qd
```

求解后：

```text
tau      = M qdd + h          # 若需要力矩
qd_des   = qd + qdd·dt        # 速度命令
q_des    = integrate(q, qd_des·dt)   # 位置命令
```

**这就是你当前阶段需要的全部 QP**。18 维变量、十几个约束，非常小。

### 5.3 为什么用加速度级

1. `xdd = J qdd + Jdot qd` 对 `qdd` 是**线性**的 → 任务和约束都是线性/二次的 → 凸 QP。
2. 力矩通过 `tau = M qdd + h` 事后算出，或作为线性约束加入。
3. 接触力、摩擦锥、力矩限位都能自然表达。
4. 输出 `qdd` 后可以积分成 `q_des/qd_des` 给 PD 伺服，也可以直接算 `tau` 给力矩伺服。

### 5.4 求解后怎么用

| 伺服类型 | WBC 输出 | 说明 |
|---|---|---|
| 位置/速度 PD（你的 Isaac） | `q_des, qd_des` | 由 `qdd` 积分；PD 负责底层跟踪 |
| 力矩伺服 | `tau = M qdd + h` | 需要高频（500–1000 Hz） |
| 混合 | `q_des, qd_des, tau_ff` | 位置 PD + 前馈力矩（Isaac 可加 effort 或目标偏置） |

**关键**：WBC 的输出接口必须和伺服输入匹配。你现在是位置/速度 PD，所以优先输出 `q_des/qd_des`。

---

## 6. 质心动力学（Centroidal，选读）

浮动基平衡常用**质心动力学**降维：

```text
A(q) qdd + Adot(q, qd) qd = [m g; 0] + Σ_i [ f_i ; p_i × f_i ]
```

- `A(q)`：centroidal map（6 × nv），Pinocchio 里 `data.Ag`。
- `h_G = A(q) v`：质心动量（6D），`data.hg`。
- `Adot(q,qd)`：`data.dAg`，由 `computeCentroidalMapTimeVariation` 计算。
- 右端是重力 + 接触力产生的 6D 合力/合力矩（在质心、世界对齐系）。

用途：

- 人形/四足的平衡控制：先解 6D 质心任务 + 接触力分配，再映射到关节力矩；
- 你的固定基座阶段**不需要**；以后解锁轮式底盘时会用到。

---

## 7. 控制回路与离散实现

一个典型的 WBC 周期（固定基座）：

```text
1. 读 q, qd（来自 /joint_states）
2. 采样参考 q_ref, qd_ref, qdd_ref（来自 cuRobo 轨迹，按仿真时间插值）
3. 更新运动学：
     computeJointJacobians(model, data, q)
     updateFramePlacements(model, data)
     computeJointJacobiansTimeVariation(model, data, q, qd)
     computeAllTerms(model, data, q, qd)      # M, nle
4. 构造任务：
     对每个末端/躯干：e, J, Jdot·qd, xdd*
5. 组装 QP：H, g, A_eq, b_eq, lb, ub
6. 求解 qdd
7. 输出：
     qd_des = qd + qdd·dt
     q_des  = integrate(q, qd_des·dt)
     clip 到关节限位
8. 发布 q_des, qd_des 给 Isaac/真机
9. 记录 log：q, qd, q_des, qdd, tau, 误差
```

频率建议：

- WBC：200–500 Hz（18 DoF 的 QP 很小，Python + OSQP 可以到 500 Hz）。
- PD 伺服：物理步 120–240 Hz（Isaac），真机驱动器通常 1 kHz。
- 规划：10–50 Hz。
- 轨迹插值在 WBC 里做，或由 JTC 做。

---

## 8. 稳定性与工程问题

1. **任务增益与伺服带宽匹配**：WBC 的 `Kp` 不要高到超过 PD 伺服能跟踪的频率；否则 WBC 会“和伺服打架”。
2. **不要双重积分**：WBC 输出 `q_des` 后，PD 已经闭环；不要在外层再叠一个位置积分器。
3. **速度前馈**：只给 `q_des` 不给 `qd_des`，PD 的阻尼会刹车；轨迹的 `qd` 要一起给。
4. **重力稳态误差**：固定基座下手臂/躯干重力全由关节承担，PD 稳态误差约 `g/Kp`；加 `tau_g` 前馈。
5. **奇异位形**：任务雅可比降秩时，QP 仍可解（有正则项），但任务精度下降；可用阻尼最小二乘或降低权重。
6. **可行性**：硬任务 + 硬限位可能无解；给任务留 slack，或把低优先级任务改成软约束。
7. **接触切换**（浮动基）：摆动腿要从接触约束里移除，否则 QP 会把脚“粘”在地上。
8. **延迟**：从读状态到命令生效有 1–3 个周期延迟；高速时要用状态预测或降低增益。
9. **数值**：`M` 只填上三角，记得对称化；四元数定期归一化；角度 wrap-around。
10. **日志**：记录 `q/qd/q_des/qdd/tau/任务误差/求解时间/有效约束`，否则调参靠猜。

---

## 9. Pinocchio 对应 API 速查

| 量 | API |
|---|---|
| 固定基座模型 | `pin.buildModelFromUrdf(path)` |
| 浮动基座模型 | `pin.buildModelFromUrdf(path, pin.JointModelFreeFlyer())` |
| 正运动学 | `pin.forwardKinematics(model, data, q, qd, qdd)` |
| 更新 frame | `pin.updateFramePlacements(model, data)` |
| 所有关节雅可比 | `pin.computeJointJacobians(model, data, q)` |
| frame 雅可比 | `pin.getFrameJacobian(model, data, fid, rf)` / `pin.computeFrameJacobian(...)` |
| 雅可比时间导数 | `pin.computeJointJacobiansTimeVariation(model, data, q, qd)` + `pin.getFrameJacobianTimeVariation(model, data, fid, rf)` |
| 质量矩阵 | `pin.crba(model, data, q)` → `data.M`（上三角） |
| 非线性项 | `pin.nonLinearEffects(model, data, q, qd)` / `pin.rnea(model, data, q, qd, zeros)` |
| 逆动力学 | `pin.rnea(model, data, q, qd, qdd)` → `data.tau` |
| 正动力学 | `pin.aba(model, data, q, qd, tau)` → `data.ddq` |
| 一次性算全 | `pin.computeAllTerms(model, data, q, qd)` |
| CoM | `pin.centerOfMass(model, data, q, qd, qdd)`；`pin.jacobianCenterOfMass(model, data, q)` |
| 质心映射 | `pin.computeCentroidalMap(model, data, q)` → `data.Ag` |
| 质心动量 | `pin.computeCentroidalMomentum(model, data, q, qd)` → `data.hg` |
| 李群积分 | `pin.integrate(model, q, v*dt)` |
| 李群误差 | `pin.difference(model, q1, q2)` |
| 旋转对数 | `pin.log3(R)`；`pin.log6(M)` |
| 四元数归一化 | `pin.normalize(model, q)` |
| 限位 | `model.lowerPositionLimit`、`upperPositionLimit`、`velocityLimit`、`effortLimit` |

更详细的用法和代码见 [04-pinocchio-guide.md](04-pinocchio-guide.md)。

---

## 10. 一页总结

```text
WBC = 在满足动力学/接触/限位的前提下，
      让多个任务（末端、躯干、posture、CoM）按权重或优先级跟踪参考。

固定基座简化：
  变量 qdd
  代价 Σ w ||J qdd + Jdot qd - xdd*||²
  约束 joint limits, tau = M qdd + h limits
  输出 q_des/qd_des 给 PD，或 tau 给力矩伺服。

浮动基扩展：
  加接触约束 Jc qdd + Jdotc qd = 0
  加摩擦锥 f ∈ K
  加动力学等式 M qdd + h = S^T tau + Jc^T f
```

下一篇：[02-hierarchical-control.md](02-hierarchical-control.md) 讲“多任务冲突时怎么分级”。
