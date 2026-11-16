# 05 · 在 cuRobo + Isaac + PD 系统上落地 WBC

> 目标：在**不推翻现有链路**的前提下，插入一层可在真机复用的 WBC。
> 当前链路：`cuRobo v2 规划 → ROS 2 bridge → Isaac Sim PD 关节驱动`。
> 机器人：固定基座，4 躯干 + 14 手臂 = 18 DoF。

---

## 1. 目标架构

```text
cuRobo v2（ROS 2，10–50 Hz）
   │  /plan/whole_body_trajectory  (trajectory_msgs/JointTrajectory)
   ▼
WBC 节点（ROS 2，200–500 Hz）          ← /joint_states
   │  /wbc/joint_command  (sensor_msgs/JointState: position + velocity)
   ▼
ROS 2 Bridge（仿真） / 硬件驱动（真机）
   │
   ▼
PD 关节伺服（Isaac 物理步 / 真机驱动器）
   │
   └── /joint_states, /clock, /tf ──► 回到 WBC
```

**WBC 节点的边界**：

- 输入：`/joint_states`（q, qd）、`/plan/whole_body_trajectory`（q, qd, qdd, t）。
- 输出：`/wbc/joint_command`（`position`, `velocity`，可选 `effort`）。
- 不关心底层是 Isaac 还是真机；只依赖 ROS 2 接口。

---

## 2. 三个先定下来的设计决策

### 2.1 WBC 放哪

| 方案 | 优点 | 缺点 | 建议 |
|---|---|---|---|
| ROS 2 独立节点 | 仿真/真机复用；易调试；语言自由 | 1–5 ms 通信延迟 | ✅ 推荐 |
| Isaac 内 Python 扩展 | 低延迟 | 绑定仿真，真机不可复用 | 只在验证时用 |
| C++ 实时节点 | 高频/硬实时 | 开发慢 | 后期性能不够再换 |

WBC 用 Python + Pinocchio + OSQP 起步；18 DoF 的 QP 很小，200–500 Hz 足够。真机如果要求 1 kHz，再迁 C++。

### 2.2 WBC 输出什么

| 伺服类型 | WBC 输出 | 你的情况 |
|---|---|---|
| 位置/速度 PD | `q_des, qd_des` | ✅ Isaac 是这种 |
| 力矩伺服 | `tau` | 真机若是力矩模式 |
| 混合 | `q_des, qd_des, tau_ff` | 位置 PD + 前馈 |

**建议**：WBC 输出 `position + velocity`；`effort` 先留空。等需要重力/动力学前馈，再加 `effort` 或切力矩模式。

### 2.3 频率

| 层 | 建议频率 | 说明 |
|---|---|---|
| cuRobo 重规划 | 10–30 Hz | 不需要每周期 |
| 轨迹输出 | 50–100 Hz | 稀疏 waypoints |
| WBC | 200–500 Hz | 18 维 QP |
| PD（Isaac） | 120–240 Hz | 物理步；可 substep |
| PD（真机） | 1 kHz | 驱动器内部 |
| `/joint_states` | 200–1000 Hz | 越高越好 |

WBC 频率低于物理步时，Isaac 保持上一个目标（零阶保持），没问题；但目标序列要平滑。

---

## 3. ROS 2 接口设计

### 3.1 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Isaac/真机 → WBC | q, qd, effort |
| `/plan/whole_body_trajectory` | `trajectory_msgs/JointTrajectory` | cuRobo → WBC | q, qd, qdd, time_from_start |
| `/wbc/joint_command` | `sensor_msgs/JointState` | WBC → Isaac/真机 | position, velocity |
| `/wbc/enable` | `std_msgs/Bool` | 上层 → WBC | 使能/急停 |
| `/clock` | `rosgraph_msgs/Clock` | Isaac → 所有 | `use_sim_time:=true` |

### 3.2 消息约定

- `JointState.name`：固定顺序，和 Isaac 关节顺序/控制器一致。
- `position`：`q_des`，长度 = 关节数。
- `velocity`：`qd_des`，长度相同。
- `effort`：可选；如果不用前馈，留空或全零。
- 时间戳：用仿真时间（`use_sim_time`），不要用 wall clock。

### 3.3 状态机

```text
IDLE ──收到轨迹──► TRACKING ──轨迹结束/超时──► HOLD
  ▲                    │
  └──── disable ───────┴── QP 失败/状态丢失 ──► FAULT（保持最后安全命令/零速）
```

- `IDLE`：不发布或发布当前位置保持。
- `TRACKING`：按轨迹参考 + WBC。
- `HOLD`：保持最后 `q_des`，`qd_des=0`。
- `FAULT`：停止发布，或发当前位置 + 零速，等待上层复位。

---

## 4. 模型一致性（最容易出问题的地方）

四个地方必须一致：

1. **cuRobo 的机器人配置**：URDF、关节名、base link 固定、碰撞球、关节限位。
2. **Pinocchio 模型**：同一个 URDF，**固定基座**（不要加 free-flyer）。
3. **Isaac Sim 的 articulation**：同一个 URDF/USD，关节顺序、零位、符号、单位。
4. **ROS 消息的关节顺序**：`JointState.name` 与 Isaac 的 articulation 顺序/控制器顺序一致。

检查清单：

- [ ] 关节名完全一致（大小写、下划线）。
- [ ] 关节顺序一致；Pinocchio 顺序 ≠ Isaac 顺序时，显式建映射。
- [ ] 零位一致（URDF 的 `<origin>` 和 Isaac 导入设置）。
- [ ] 符号一致（有些 URDF 的轴方向相反）。
- [ ] 单位一致（米/弧度）。
- [ ] 固定基座：Pinocchio 无 free-flyer；cuRobo base 固定；Isaac articulation 固定。
- [ ] `q` 长度：Pinocchio `nq` = Isaac 关节数。
- [ ] 用一组随机 `q` 分别算 FK，比较末端位置（误差 < 1e-6 m）。

**关节顺序映射**：

```python
# Pinocchio 的 q 顺序是 URDF 树遍历顺序；ROS/Isaac 可能是另一顺序。
# 对每个 1-DoF 关节，记录 idx_q / idx_v。
class JointMap:
    def __init__(self, model, ros_joint_names):
        self.q_idx, self.v_idx = [], []
        for name in ros_joint_names:
            jid = model.getJointId(name)
            jm = model.joints[jid]
            assert jm.nq == 1 and jm.nv == 1, f"{name} 不是 1-DoF 关节"
            self.q_idx.append(jm.idx_q)
            self.v_idx.append(jm.idx_v)

    def to_pin_q(self, q_ros):
        q = np.zeros(self.model.nq)
        q[self.q_idx] = q_ros
        return q

    def from_pin_q(self, q):
        return q[self.q_idx]

    def from_pin_v(self, v):
        return v[self.v_idx]
```

多自由度关节（浮动基、球形关节）需要更复杂的映射；你当前全是 1-DoF，简单。

---

## 5. 轨迹采样与插值

cuRobo 输出的是离散 waypoints。每个 WBC 周期按当前时间采样：

```python
from dataclasses import dataclass

@dataclass
class TrajPoint:
    t: float
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray

def sample_trajectory(points, t):
    """线性插值位置；速度/加速度取相邻点线性插值或直接使用消息值。"""
    if t <= points[0].t:
        return points[0]
    if t >= points[-1].t:
        return points[-1]

    # 二分/线性查找区间
    for i in range(len(points) - 1):
        if points[i].t <= t <= points[i + 1].t:
            dt = points[i + 1].t - points[i].t
            s = (t - points[i].t) / dt
            q = (1 - s) * points[i].q + s * points[i + 1].q
            qd = (1 - s) * points[i].qd + s * points[i + 1].qd
            qdd = (1 - s) * points[i].qdd + s * points[i + 1].qdd
            return TrajPoint(t, q, qd, qdd)
```

更平滑的做法：

- **三次 Hermite**：用端点 `q, qd`，保证速度连续；
- **五次多项式**：用端点 `q, qd, qdd`，保证加速度连续；
- **JTC**：如果走 `ros2_control`，让 `joint_trajectory_controller` 做插值，WBC 只订阅参考或直接由 JTC 输出。

**注意**：

- cuRobo 的 `dt` 可能不均匀；按 `time_from_start` 找区间。
- 轨迹结束/超时：进入 `HOLD`，不要继续外推。
- 重规划：新轨迹从当前时间开始，做 0.1–0.3 s 的混合，避免跳变。

---

## 6. WBC 的 QP 实现（固定基座）

### 6.1 数学形式

变量 `x = qdd ∈ R^nv`。

```text
min_x  0.5 x^T H x + g^T x
s.t.   l ≤ A x ≤ u
```

- 任务：`H += w J^T J`，`g += -w J^T b`，其中 `b = xdd* - Jdot qd`。
- 力矩：`A_tau = M`，`l_tau = tau_min - h`，`u_tau = tau_max - h`。
- 速度：`A_vel = dt·I`，`l_vel = v_min - qd`，`u_vel = v_max - qd`。
- 位置（近似）：`A_pos = 0.5·dt²·I`，`l_pos = q_min - q - qd·dt`，`u_pos = q_max - q - qd·dt`。
- 加速度：box `qdd_min ≤ x ≤ qdd_max`。

### 6.2 完整代码骨架

```python
import numpy as np
import pinocchio as pin
import osqp
import scipy.sparse as spa
from dataclasses import dataclass


@dataclass
class TaskRef:
    frame_id: int
    p_des: np.ndarray          # 3
    R_des: np.ndarray          # 3x3
    v_des: np.ndarray          # 6, LOCAL_WORLD_ALIGNED
    a_des: np.ndarray          # 6
    w_pos: float = 1.0
    w_rot: float = 0.5


class FixedBaseWBC:
    def __init__(self, urdf_path, ros_joint_names, torso_frame_name,
                 ee_frame_names, dt=0.005):
        self.model = pin.buildModelFromUrdf(urdf_path)   # 固定基座
        self.data = self.model.createData()
        self.nv = self.model.nv
        self.dt = dt

        self.torso_fid = self.model.getFrameId(torso_frame_name)
        self.ee_fids = [self.model.getFrameId(n) for n in ee_frame_names]

        # 关节顺序映射（Pinocchio <-> ROS）
        self.q_idx, self.v_idx = [], []
        for name in ros_joint_names:
            jid = self.model.getJointId(name)
            jm = self.model.joints[jid]
            assert jm.nq == 1 and jm.nv == 1
            self.q_idx.append(jm.idx_q)
            self.v_idx.append(jm.idx_v)
        self.ros_joint_names = ros_joint_names

        # 限位
        self.q_min = self.model.lowerPositionLimit.copy()
        self.q_max = self.model.upperPositionLimit.copy()
        self.v_max = self.model.velocityLimit.copy()
        self.tau_max = np.where(
            np.isfinite(self.model.effortLimit),
            self.model.effortLimit, 1000.0)
        self.qdd_max = 100.0 * np.ones(self.nv)   # 按机器人调整

        # 增益
        self.Kp_ee, self.Kd_ee = 200.0, 2 * np.sqrt(200.0)
        self.Kp_rot, self.Kd_rot = 100.0, 2 * np.sqrt(100.0)
        self.Kp_pos, self.Kd_pos = 20.0, 2 * np.sqrt(20.0)

        # 躯干期望姿态（可由上层/参考更新）
        self.R_torso_des = np.eye(3)

        # 用于从 q_ref 反算末端/躯干期望位姿
        self.data_ref = self.model.createData()

    def to_pin_q(self, q_ros):
        q = np.zeros(self.model.nq)
        q[self.q_idx] = q_ros
        return q

    def to_pin_v(self, v_ros):
        v = np.zeros(self.model.nv)
        v[self.v_idx] = v_ros
        return v

    # ---------- 任务 ----------
    def _pose_task(self, ref: TaskRef, w_pos, w_rot):
        d = self.data
        oMf = d.oMf[ref.frame_id]
        e = np.concatenate([
            ref.p_des - oMf.translation,
            pin.log3(ref.R_des @ oMf.rotation.T),
        ])
        J = pin.getFrameJacobian(
            self.model, d, ref.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        dJ = pin.getFrameJacobianTimeVariation(
            self.model, d, ref.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        vel = pin.getFrameVelocity(
            self.model, d, ref.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector

        Kp = np.diag(np.array([self.Kp_ee] * 3 + [self.Kp_rot] * 3))
        Kd = np.diag(np.array([self.Kd_ee] * 3 + [self.Kd_rot] * 3))
        a_star = ref.a_des + Kp * e + Kd * (ref.v_des - vel)
        b = a_star - dJ @ self._qd
        return J, b, np.array([w_pos] * 3 + [w_rot] * 3)

    # ---------- 主步骤 ----------
    def step(self, q, qd, refs, q_ref, qd_ref, qdd_ref):
        self._qd = qd
        d = self.data

        # 运动学 + 动力学
        pin.computeAllTerms(self.model, d, q, qd)     # data.M, data.nle, data.J...
        pin.updateFramePlacements(self.model, d)       # computeAllTerms 不更新 oMf
        pin.computeJointJacobiansTimeVariation(self.model, d, q, qd)

        M = np.triu(d.M) + np.triu(d.M, 1).T
        h = d.nle.copy()

        # 任务
        H = np.zeros((self.nv, self.nv))
        g = np.zeros(self.nv)
        for ref in refs:
            J, b, w = self._pose_task(ref, ref.w_pos, ref.w_rot)
            W = np.diag(w)
            H += J.T @ W @ J
            g += -J.T @ W @ b

        # 躯干姿态任务（只取角速度 3 行）
        oMt = d.oMf[self.torso_fid]
        e_R = pin.log3(self.R_torso_des @ oMt.rotation.T)
        Jt = pin.getFrameJacobian(
            self.model, d, self.torso_fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[3:6, :]
        dJt = pin.getFrameJacobianTimeVariation(
            self.model, d, self.torso_fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[3:6, :]
        w_ang = pin.getFrameVelocity(
            self.model, d, self.torso_fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).angular
        a_t = self.Kp_rot * e_R + self.Kd_rot * (-w_ang)
        b_t = a_t - dJt @ qd
        H += Jt.T @ Jt
        g += -Jt.T @ b_t

        # posture 任务：跟踪规划 q_ref（低权重）
        e_p = pin.difference(self.model, q_ref, q)
        b_p = qdd_ref + self.Kp_pos * e_p + self.Kd_pos * (qd_ref - qd)
        H += 1.0 * np.eye(self.nv)
        g += -1.0 * b_p

        # 正则
        H += 1e-6 * np.eye(self.nv)

        # 约束：力矩 / 速度 / 位置 / 加速度
        A_list, l_list, u_list = [], [], []
        A_list.append(M); l_list.append(-self.tau_max - h); u_list.append(self.tau_max - h)
        A_list.append(self.dt * np.eye(self.nv))
        l_list.append(-self.v_max - qd); u_list.append(self.v_max - qd)
        A_list.append(0.5 * self.dt**2 * np.eye(self.nv))
        l_list.append(self.q_min - q - qd * self.dt)
        u_list.append(self.q_max - q - qd * self.dt)
        A = np.vstack(A_list)
        l = np.concatenate(l_list)
        u = np.concatenate(u_list)

        # 加速度 box 合并进 l/u（A 的最后 nv 行是单位阵）
        # 简化：把加速度约束单独拼在 A 里
        A = np.vstack([A, np.eye(self.nv)])
        l = np.concatenate([l, -self.qdd_max])
        u = np.concatenate([u, self.qdd_max])

        # 求解
        P = spa.csc_matrix(np.triu(H))   # OSQP 只读上三角
        A_sp = spa.csc_matrix(A)
        prob = osqp.OSQP()
        prob.setup(P=P, q=g, A=A_sp, l=l, u=u,
                   verbose=False, eps_abs=1e-6, eps_rel=1e-6, max_iter=4000)
        res = prob.solve()
        if res.x is None:
            return None, None, None

        qdd = res.x
        qd_des = np.clip(qd + qdd * self.dt, -self.v_max, self.v_max)
        q_des = pin.integrate(self.model, q, qd_des * self.dt)
        q_des = np.clip(q_des, self.q_min, self.q_max)

        # 转成 ROS 顺序
        return q_des[self.q_idx], qd_des[self.v_idx], qdd
```

几点说明：

- `computeAllTerms` 已经算了 `data.J`（LOCAL），`getFrameJacobian` 用它；`computeJointJacobiansTimeVariation` 算 `data.dJ`。
- `data.M` 上三角，必须对称化。
- `osqp.setup` 每个周期重建，18 维没问题；要更快就复用 problem 并 `update`（P/A 稀疏结构不变）。
- 位置约束用二阶近似，可能偏保守；也可以只做速度/力矩约束，输出后 clip。
- `q_ref/qd_ref/qdd_ref` 来自轨迹采样；posture 任务把它们作为参考。
- 躯干姿态任务用 `R_torso_des` 作为成员变量，实际从参考/固定值来。

### 6.3 输出与发布

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


def parse_trajectory(msg):
    points = []
    for p in msg.points:
        t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        q = np.array(p.positions)
        qd = np.array(p.velocities) if len(p.velocities) == len(q) else np.zeros_like(q)
        qdd = np.array(p.accelerations) if len(p.accelerations) == len(q) else np.zeros_like(q)
        points.append(TrajPoint(t, q, qd, qdd))
    return points


class WBCNode(Node):
    def __init__(self):
        super().__init__("wbc")
        self.declare_parameter("urdf", "")
        self.declare_parameter("dt", 0.005)
        # ... 读取参数、构造 self.wbc ...

        self.q = None            # Pinocchio 顺序
        self.qd = None
        self.traj_points = []
        self.t0 = None

        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.create_subscription(JointTrajectory, "/plan/whole_body_trajectory",
                                 self.on_traj, 10)
        self.pub = self.create_publisher(JointState, "/wbc/joint_command", 10)
        self.create_timer(self.wbc.dt, self.on_timer)

    def on_state(self, msg):
        # /joint_states 是 ROS 顺序；转成 Pinocchio 顺序
        self.q = self.wbc.to_pin_q(np.array(msg.position))
        self.qd = self.wbc.to_pin_v(np.array(msg.velocity))

    def on_traj(self, msg):
        self.traj_points = parse_trajectory(msg)   # 仍是 ROS 顺序
        self.t0 = self.get_clock().now()

    def build_task_refs(self, q_ref, qd_ref, qdd_ref):
        """从规划关节参考反算末端期望位姿 + 前馈速度/加速度。"""
        model, data = self.wbc.model, self.wbc.data_ref
        pin.framesForwardKinematics(model, data, q_ref)
        pin.computeJointJacobians(model, data, q_ref)
        pin.computeJointJacobiansTimeVariation(model, data, q_ref, qd_ref)

        refs = []
        for fid in self.wbc.ee_fids:
            oMf = data.oMf[fid]
            J = pin.getFrameJacobian(
                model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            dJ = pin.getFrameJacobianTimeVariation(
                model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            v_des = J @ qd_ref
            a_des = J @ qdd_ref + dJ @ qd_ref
            refs.append(TaskRef(
                fid, oMf.translation.copy(), oMf.rotation.copy(), v_des, a_des))
        return refs

    def on_timer(self):
        if self.q is None or not self.traj_points:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        ref = sample_trajectory(self.traj_points, t)

        # 轨迹是 ROS 顺序；转成 Pinocchio 顺序
        q_ref = self.wbc.to_pin_q(ref.q)
        qd_ref = self.wbc.to_pin_v(ref.qd)
        qdd_ref = self.wbc.to_pin_v(ref.qdd)

        task_refs = self.build_task_refs(q_ref, qd_ref, qdd_ref)
        q_des, qd_des, qdd = self.wbc.step(
            self.q, self.qd, task_refs, q_ref, qd_ref, qdd_ref)
        if q_des is None:
            return                      # QP 失败：保持/报错

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self.wbc.ros_joint_names
        cmd.position = q_des.tolist()
        cmd.velocity = qd_des.tolist()
        self.pub.publish(cmd)
```

> 说明：上面的 `build_task_refs` 用 `q_ref` 的 FK 作为末端/躯干期望，意味着 WBC 主要是在关节空间跟踪规划，
> 同时用笛卡尔任务提供刚度/冗余分配。如果上层直接给笛卡尔目标（而不是关节轨迹），把 `build_task_refs`
> 换成读取笛卡尔参考即可。

> 注意：`q_ref` 必须是 **Pinocchio 顺序**；`/joint_states` 和 `JointTrajectory` 是 **ROS 顺序**，
> 两者之间的映射不能省。多自由度关节（浮动基/球形）需要更复杂的映射。

> 以上代码假设：固定基座、所有关节都是 1-DoF、位置用标量表示、`nq == nv`。
> 如果 URDF 里有 `continuous` 关节（`nq=2`）或以后加浮动基，`to_pin_q`、`np.clip`、`integrate`
> 都要按李群/多自由度关节改写。

> 时间基准：上面的 `t0 = 收到消息的时刻` 适合仿真/流式执行；真机建议用轨迹的
> `header.stamp` 或 action 的 goal 时间，并配合 `use_sim_time` / 硬件时钟做对齐。

---

## 7. Isaac 侧改造

### 7.1 关节驱动配置

USD 里每个 joint 的 Drive：

- `driveMode = position`（或 position + velocity）
- `stiffness = Kp`，`damping = Kd`
- `maxForce = 力矩上限`
- 目标：`position target = q_des`，`velocity target = qd_des`

Isaac 的隐式 PD：

```text
tau = Kp (q_des - q) + Kd (qd_des - qd)
```

**必须同时设速度目标**，否则 `qd_des=0` 会让阻尼项刹车。

### 7.2 ROS 2 Bridge

OmniGraph 典型连接：

```text
ROS2PublishJointState  ──► /joint_states
ROS2PublishClock       ──► /clock
ROS2SubscribeJointState ◄── /wbc/joint_command
        │ position/velocity
        ▼
ArticulationController ──► joint drives
```

注意：

- `ROS2SubscribeJointState` 的数组顺序必须和 articulation 的关节顺序一致；如果消息按名字映射，确认 bridge 的配置。
- bridge 的 `use_sim_time` 和 `/clock` 要开。
- 命令频率低于物理步时，drive 保持上一个目标；这是正常的零阶保持。
- 如果 bridge 不支持 `JointState` 的 velocity，改用 `Float64MultiArray` 或自定义消息。

### 7.3 重力前馈（可选）

PD 单独对抗重力会有稳态误差。两种做法：

1. **目标偏置**：`q_cmd = q_des + tau_g / Kp`，`tau_g = pin.computeGeneralizedGravity(model, data, q)`。
2. **effort 前馈**：如果 Isaac 支持在 drive 之外 `set_joint_efforts(tau_g)`，直接叠加；注意不要和 drive 的 PD 重复。

先加 `tau_g`，再考虑 `M qdd + h`。

---

## 8. 真机迁移

WBC 节点本身不变，替换桥接和伺服：

```text
仿真：WBC → ROS 2 Bridge → Isaac PD
真机：WBC → 硬件驱动（EtherCAT/CAN/ROS 2 hardware_interface）→ 关节伺服
```

需要补的：

1. **关节映射/标定**：零位、符号、减速比、编码器偏移。
2. **伺服接口**：位置/速度/力矩模式；命令周期；使能/急停。
3. **重力/摩擦前馈**：`tau_g`、库仑摩擦、armature。
4. **系统辨识**：质量、CoM、惯量、摩擦；至少把 `model.armature` 和 `rotorInertia` 配对。
5. **安全层**：看门狗、命令超时、限位、速度/加速度限幅、急停、碰撞检测。
6. **延迟补偿**：测量从状态采样到命令生效的延迟；高速时用预测或降低增益。
7. **低增益试跑**：悬吊/空载，先位置保持，再小幅度轨迹，再全速。
8. **日志**：q/qd/q_des/tau/误差/延迟，方便复盘。

**真机与仿真的差异**：

- 关节摩擦/间隙/柔性 → 跟踪误差、抖动；
- 通信延迟/丢包 → 用时间戳和 watchdog；
- 力矩限幅/驱动器保护 → WBC 的力矩约束要保守；
- 传感器噪声 → 速度低通、任务增益降低；
- 重力/负载 → 前馈和辨识。

---

## 9. 安全与测试

### 9.1 安全

- WBC 输出前：`q_des/qd_des` clip 到限位；`qdd` 限幅；力矩约束。
- watchdog：`/joint_states` 超过 50 ms 没更新 → HOLD/FAULT。
- 轨迹超时/结束 → HOLD。
- QP 失败 → 保持上一条安全命令，记录错误。
- 急停：`/wbc/enable=false` → 停止发布或发零速。
- 真机：使能信号、刹车、限位开关、电流/温度保护。

### 9.2 测试顺序

1. **模型一致性**：随机 q 下 Pinocchio FK vs Isaac FK。
2. **单关节**：正弦位置/速度目标，验证符号、零位、PD。
3. **静态姿态**：WBC 保持 q_ref，验证不发散。
4. **末端小轨迹**：笛卡尔圆/直线，看跟踪误差。
5. **cuRobo 轨迹**：静态目标 → 慢速轨迹 → 快速轨迹。
6. **扰动**：手动推、加负载，看恢复。
7. **真机**：悬吊 → 低增益 → 小幅度 → 全速。

指标：

- 关节跟踪误差 RMS/max；
- 末端笛卡尔误差；
- 关节限位余量；
- 力矩饱和比例；
- QP 求解时间/失败率；
- 通信延迟。

---

## 10. 与 cuRobo 的配合

- cuRobo 负责全局无碰撞、满足关节限位的轨迹；
- WBC 负责局部跟踪、约束、冗余；
- **限制 WBC 偏离规划的量**：`||q_des - q_ref||` 或末端偏差超过阈值时，降低任务权重/回退到规划；
- 重规划时做 0.1–0.3 s 混合；
- 如果 cuRobo 输出只有 q 没有 qd/qdd，用五次插值并假设端点速度/加速度为零。

---

## 11. 分阶段实施清单

- [ ] **P0 模型对齐**：同一 URDF，Pinocchio/Isaac/cuRobo 关节名、顺序、零位、符号、单位一致；FK 对比通过。
- [ ] **P1 轨迹直通**：cuRobo → bridge → PD，验证静态/慢速轨迹。
- [ ] **P2 WBC 节点**：订阅状态/轨迹，发布 `q_des/qd_des`；先只做 posture 跟踪。
- [ ] **P3 加任务**：左右末端 6D + 躯干 3D；加权 QP；对比笛卡尔误差。
- [ ] **P4 加约束**：位置/速度/加速度/力矩限位；QP 失败处理。
- [ ] **P5 加前馈**：`tau_g`，再考虑 `M qdd + h`。
- [ ] **P6 真机**：替换桥接、标定、安全层、低增益试跑。
- [ ] **P7 底盘**：浮动基 + 接触 + centroidal（以后）。

---

## 12. 常见问题

**Q：WBC 输出位置还是力矩？**
A：你的 Isaac 是位置/速度 PD，输出 `q_des/qd_des`；真机若力矩模式，输出 `tau = M qdd + h`。

**Q：WBC 频率一定要比 PD 高吗？**
A：不需要。PD 在物理步/驱动器内部高频运行；WBC 200 Hz 足够，Isaac 保持目标。

**Q：QP 失败怎么办？**
A：先检查约束是否矛盾（力矩/位置限位太紧）；给任务加 slack；失败时保持上一条安全命令。

**Q：需要 HQP 吗？**
A：固定基座先用加权 QP；任务冲突明显/安全关键再上。

**Q：需要质心/接触吗？**
A：固定基座不需要；解锁轮式底盘再加。

**Q：Python 够快吗？**
A：18 DoF、几十约束，Python + OSQP 200–500 Hz 没问题；不够再 C++。

---

下一篇：[06-knowledge-and-roadmap.md](06-knowledge-and-roadmap.md) 给出知识体系、资料清单和长期路线。
