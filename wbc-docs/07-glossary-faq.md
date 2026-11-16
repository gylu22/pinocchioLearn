# 07 · 术语表与常见问题（FAQ）

> 用来快速查词、纠正误解、面试前复习。
> 术语按主题分组，英文保留，便于读论文/代码。

---

## 1. 术语表

### 1.1 运动学/动力学

| 术语 | 英文 | 含义 |
|---|---|---|
| 广义坐标 | generalized coordinates `q` | 描述机器人构型的向量 |
| 广义速度 | generalized velocity `v` | 切空间速度，`dim = nv` |
| 自由度 | DoF | 独立运动维数；固定基座 `nv = 关节数` |
| 浮动基 | floating base | 基座有 6 DoF，`nq ≠ nv` |
| 正运动学 | forward kinematics (FK) | `q → 末端/关节位姿` |
| 雅可比 | Jacobian `J` | `xd = J qd` |
| 雅可比导数 | Jacobian dot `J̇` | `xdd = J qdd + J̇ qd` |
| 质量矩阵 | mass matrix `M` | 关节空间惯量 |
| 非线性项 | nonlinear effects `h` | `C(q,qd)qd + g(q)` |
| 逆动力学 | inverse dynamics (RNEA) | `(q,qd,qdd) → tau` |
| 正动力学 | forward dynamics (ABA) | `(q,qd,tau) → qdd` |
| 复合刚体算法 | CRBA | 算 `M` |
| 空间代数 | spatial algebra | 6D Motion/Force 运算 |
| 质心 | center of mass (CoM) | 质量加权平均位置 |
| 质心映射 | centroidal map `A` | `h_G = A(q) v` |
| 质心动量 | centroidal momentum `h_G` | 6D 动量 |

### 1.2 控制

| 术语 | 英文 | 含义 |
|---|---|---|
| 任务 | task | 希望控制的输出，如末端位姿、CoM |
| 约束 | constraint | 必须满足的硬条件 |
| 任务空间 | task space | 任务所在的笛卡尔/特征空间 |
| 零空间 | null space | 不改变任务输出的关节运动子空间 |
| 伪逆 | pseudo-inverse `J⁺` | 最小范数解 |
| 动力学一致伪逆 | dynamically consistent inverse `J#` | M 加权伪逆 |
| 操作空间控制 | operational space control (OSC) | 任务空间力 → 关节力矩 |
| 计算力矩 | computed torque | `tau = M(qdd* + PD) + h` |
| 阻抗控制 | impedance control | 模拟弹簧-阻尼 |
| 导纳控制 | admittance control | 力 → 位置修正 |
| 分层/分级控制 | hierarchical control | 严格优先级 |
| HQP | hierarchical QP | 分层二次规划 |
| 加权 QP | weighted QP | 单层软优先级 |
| 前馈 | feedforward | 用模型提前补偿 |
| 反馈 | feedback | 用误差修正 |
| 增益 | gain `Kp/Kd` | 比例/微分系数 |
| 临界阻尼 | critical damping | `Kd = 2√Kp`（按惯量调整） |

### 1.3 接触/平衡

| 术语 | 英文 | 含义 |
|---|---|---|
| 接触约束 | contact constraint | 接触点不滑动 |
| 摩擦锥 | friction cone | `|f_t| ≤ μ f_n` |
| 四棱锥 | linearized friction pyramid | 摩擦锥的线性近似 |
| 支撑多边形 | support polygon | 接触点凸包 |
| ZMP | zero moment point | 地面反力合力矩为零的点 |
| CoP | center of pressure | 压力中心 |
| 接触切换 | contact switching | 支撑/摆动切换 |
| Baumgarte 稳定 | Baumgarte stabilization | 用位置/速度误差修正约束漂移 |

### 1.4 软件/工具

| 术语 | 含义 |
|---|---|
| Pinocchio | 刚体动力学库（模型/FK/J/M/h/导数） |
| TSID | 基于 Pinocchio 的分层 WBC 库 |
| Crocoddyl | 多接触最优控制/DDP |
| Aligator | ProxDDP 轨迹优化 |
| Pink | 基于 Pinocchio 的分层 IK |
| OSQP | 稀疏 QP 求解器 |
| ProxSuite | 一阶/二阶 QP 求解器 |
| `ros2_control` | ROS 2 控制器框架 |
| JTC | `joint_trajectory_controller` |
| `use_sim_time` | 使用 `/clock` 仿真时间 |

---

## 2. FAQ

### Q1：WBC 和 MPC 有什么区别？

- WBC 通常是**单步**的：每个周期解一个瞬时优化，跟踪当前任务。
- MPC 是**多步**的：在预测时域内优化未来状态/控制序列，能预判、处理接触切换。
- 两者可以级联：MPC 出参考，WBC 跟踪；也可以统一成一个 MPC。

### Q2：WBC 和 IK 有什么区别？

- IK 只解运动学，输出 `q` 或 `qd`，不涉及质量/力矩。
- WBC 解动力学/约束，输出 `qdd` 或 `tau`，能处理力矩限位、接触力、动态前馈。
- 固定基座、慢速时，IK 往往够用；高速/重载/接触时 WBC 更合适。

### Q3：为什么 WBC 通常在加速度级，而不是力矩级？

- 任务 `xdd = J qdd + J̇ qd` 对 `qdd` 线性 → 凸 QP。
- 力矩通过 `tau = M qdd + h` 事后算，或作为线性约束。
- 直接优化 `tau` 也可以，但任务约束会变成 `J M^{-1} tau`，仍然线性，只是数值/约束表达不同。加速度级更直观。

### Q4：WBC 输出位置还是力矩？

- 看伺服：位置/速度 PD → 输出 `q_des/qd_des`；力矩伺服 → 输出 `tau`。
- 位置 PD + 前馈是折中：`q_des/qd_des` + `tau_ff`（重力/惯性）。
- 不要给位置伺服直接叠加一个没有建模的力矩，容易打架。

### Q5：权重怎么调？

- 按“期望误差量级”归一化：位置（m）、姿态（rad）、posture（rad）不要直接比。
- 先给每个任务单独跑，记录误差量级；再按 `w ≈ 1/允许误差²` 设初值。
- 任务冲突时，优先保证安全/接触，再调任务。
- 调不动就上分层。

### Q6：QP 不可行怎么办？

- 检查约束是否矛盾：力矩限位太紧、位置限位太近、加速度限幅太小。
- 给低优先级任务加 slack（软约束）。
- 按优先级从低到高放松。
- 保持上一条安全命令，记录错误，不要静默下发无效解。

### Q7：为什么 `data.M` 要对称化？

- `crba` 只填上三角，下三角可能是旧值/垃圾。
- QP、求逆、特征值都需要完整对称矩阵。
- `M = triu(M) + triu(M,1).T`。

### Q8：`nq` 和 `nv` 为什么不一样？

- 浮动基用四元数表示姿态：`nq = 7 + nj`，`nv = 6 + nj`。
- 连续关节可能用 `cos/sin` 表示：`nq = 2`，`nv = 1`。
- 所以 `q` 不能直接加减，要用 `integrate/difference`。

### Q9：任务雅可比用哪个 `ReferenceFrame`？

- 笛卡尔任务：`LOCAL_WORLD_ALIGNED`（原点在 frame，轴向世界）。
- 接触力/摩擦锥：`LOCAL`（力在接触系）。
- 只要 `J`、`f`、误差三者坐标系一致即可；不同任务可以用不同参考系。

### Q10：为什么高速时不能省 `J̇ qd`？

- 任务加速度 `xdd = J qdd + J̇ qd`；`J̇ qd` 是科氏/离心项。
- 高速/大范围运动时它与 `J qdd` 同量级，省掉会导致跟踪偏差甚至不稳定。

### Q11：Python 够快吗？

- 18 DoF、几十约束的 QP，Python + OSQP 200–500 Hz 没问题。
- 瓶颈通常在 ROS 通信、日志、Python GIL，不在 QP。
- 需要 1 kHz/硬实时再迁 C++。

### Q12：仿真跑通了，真机为什么不行？

- 模型差异：质量/惯量/摩擦/armature/零位/符号。
- 延迟/抖动/丢包。
- 驱动器限幅/保护/死区。
- 传感器噪声。
- 安全策略不同。
- 解决：系统辨识、低增益试跑、前馈、watchdog、日志复盘。

### Q13：需要给 WBC 加积分项吗？

- 通常不需要；PD 伺服本身有低频增益。
- 加积分容易在饱和/接触切换时 windup。
- 稳态误差用前馈（重力/摩擦）解决，而不是积分。

### Q14：WBC 频率和 PD 频率谁高？

- PD 在物理步/驱动器内部高频（120 Hz–1 kHz）。
- WBC 200–500 Hz 即可；低于 PD 频率没问题，驱动器保持上一个目标。
- WBC 太低（<50 Hz）会导致目标阶梯感。

### Q15：固定基座需要接触/质心吗？

- 不需要。基座固定，重力由基座/关节承担，没有平衡问题。
- 质心/接触是浮动基/轮式底盘才需要。

### Q16：什么时候该上 HQP？

- 任务冲突明显，加权调不好；
- 安全/接触必须严格优先；
- 多任务、多接触、人形/四足。
- 固定基座 18 DoF 通常加权 QP 足够。

### Q17：cuRobo 规划的和 WBC 执行的不一致怎么办？

- 限制 WBC 偏离规划的量（关节/末端偏差阈值）；
- 重规划时做轨迹混合；
- 把 WBC 的局部修正反馈给规划层（闭环重规划）；
- 碰撞约束交给规划层，WBC 不要大幅改变构型。

### Q18：怎么验证 WBC 是对的？

- 单元测试：`tau = M qdd + h` vs `rnea`；`J qdd + J̇ qd` vs 数值差分。
- 仿真：静态保持、正弦跟踪、笛卡尔圆、扰动恢复。
- 指标：跟踪误差、限位余量、力矩饱和、QP 时间/失败率。
- 真机：悬吊、低增益、小幅度、逐步放开。

---

## 3. 常见误解

1. **“WBC 就是加一个 QP”**：QP 只是求解器；关键是任务/约束建模和优先级。
2. **“有了 WBC 就不需要 PD”**：WBC 是上层，PD 是底层伺服；两者互补。
3. **“WBC 一定要浮动基/接触”**：固定基座可以大幅简化。
4. **“权重越大越优先”**：量纲/尺度不同，权重不能直接比较；严格优先级要用分层。
5. **“仿真跑通就能上真机”**：模型、延迟、摩擦、安全都要补。
6. **“控制频率越高越好”**：稳定性和延迟匹配更重要；WBC 200 Hz + PD 1 kHz 很常见。

---

## 4. 面试/自测题

1. 写出浮动基动力学方程，并说明每一项的维数。
2. 推导 `xdd = J qdd + J̇ qd`。
3. 解释零空间投影 `N = I - J⁺J` 的几何意义。
4. 动态一致伪逆和普通伪逆的区别？
5. 加权 QP 和 HQP 的优缺点？
6. 摩擦锥为什么要线性化？四棱锥和 SOCP 的区别？
7. `data.M` 为什么只填上三角？
8. 浮动基为什么 `nq ≠ nv`？怎么积分？
9. WBC 输出位置和输出力矩，分别怎么和 PD 伺服配合？
10. 真机迁移最常见的三个问题？

---

回到 [README.md](README.md)。
