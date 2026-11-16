# 04 · Pinocchio 实战手册（面向 WBC）

> 目标：把 WBC 需要的量在 Pinocchio 里一一对应起来，给出可直接跑的代码片段和常见坑。
> 版本：Pinocchio 4.x（当前仓库 devel）。C++ 与 Python 接口同名。

---

## 1. 安装与版本

```bash
# conda（推荐）
conda install -c conda-forge pinocchio

# pip（Linux）
pip install pin

# 或从源码构建（见仓库 development/build.md）
```

检查：

```python
import pinocchio as pin
print(pin.__version__)
```

WBC 只需要核心库（Eigen + Boost），URDF 解析需要 `urdfdom`，碰撞需要 `coal`。

---

## 2. Model / Data

Pinocchio 的核心范式：**Model 只读，Data 存所有计算结果**。所有算法签名都是 `f(model, data, args...)`。

```python
import pinocchio as pin

# 固定基座（你的场景）
model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

print(model.nq, model.nv, model.njoints, model.nframes)
# 固定基座且全是 1-DoF 关节时 nq == nv
```

```python
# 浮动基座（以后解锁轮式底盘）
model_f = pin.buildModelFromUrdf("robot.urdf", pin.JointModelFreeFlyer())
# nq = 7 + nj, nv = 6 + nj
```

常用字段：

| 字段 | 含义 |
|---|---|
| `model.nq` / `model.nv` | 配置/速度维数 |
| `model.njoints` | 关节数（含 universe） |
| `model.nframes` | frame 数 |
| `model.names[jid]` | 关节名 |
| `model.frames[fid]` | frame（name, parentJoint, placement, type） |
| `model.parents[jid]` | 父关节 |
| `model.inertias[jid]` | 关节惯量 |
| `model.jointPlacements[jid]` | 关节相对父关节的 placement |
| `model.lowerPositionLimit` / `upperPositionLimit` | 位置限位 |
| `model.velocityLimit` | 速度限位 |
| `model.effortLimit` | 力矩限位 |
| `model.armature` | 关节 armature（加到 M 对角） |
| `model.rotorInertia` / `rotorGearRatio` | 电机转子惯量/减速比 |
| `model.gravity` | 重力（默认 `[0,0,-9.81]`） |

**固定基座注意**：URDF 里没有浮动基时，`buildModelFromUrdf(path)` 的根是固定的；URDF 里的 `fixed` joint 会被转成 frame，不会进入 `njoints`。所以你的“4 躯干 + 14 手臂”如果都是 revolute/prismatic，`nv=18`。

---

## 3. 正运动学与 frame

```python
q = pin.neutral(model)
pin.forwardKinematics(model, data, q)          # 只算 oMi
pin.updateFramePlacements(model, data)         # 算 oMf

# 关节 i 的 placement（世界系）
oMi = data.oMi[jid]
# frame fid 的 placement（世界系）
oMf = data.oMf[fid]
```

只关心 frame 时：

```python
pin.framesForwardKinematics(model, data, q)    # 只更新 frame
```

frame id：

```python
fid = model.getFrameId("left_ee")
jid = model.getJointId("waist_yaw")
```

**建议**：接触点、末端、躯干都建成 frame（URDF 里加 link + fixed joint，或 `model.addFrame`），WBC 里统一用 frame id。

---

## 4. 雅可比

### 4.1 关节雅可比

```python
pin.computeJointJacobians(model, data, q)      # data.J: 所有关节 6×nv（LOCAL）
J_joint = pin.getJointJacobian(model, data, jid, pin.ReferenceFrame.LOCAL)
```

### 4.2 frame 雅可比

```python
# 先算 computeJointJacobians
J = pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

# 或者一步到位（内部会算）
J = pin.computeFrameJacobian(model, data, q, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
```

`ReferenceFrame`：

| 值 | 原点 | 轴向 | 适用 |
|---|---|---|---|
| `LOCAL` | frame 原点 | frame 自身 | 接触力、局部任务 |
| `WORLD` | 世界原点 | 世界轴 | 少用 |
| `LOCAL_WORLD_ALIGNED` | frame 原点 | 世界轴 | 笛卡尔任务（推荐） |

6D 雅可比的行顺序：`[linear; angular]`。

### 4.3 雅可比时间导数

```python
pin.computeJointJacobiansTimeVariation(model, data, q, v)   # data.dJ
dJ = pin.getFrameJacobianTimeVariation(
    model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
)
```

任务加速度：

```python
xdd = J @ qdd + dJ @ v
```

`dJ @ v` 是偏置项，高速时不能省。

### 4.4 frame 速度/加速度

```python
vel = pin.getFrameVelocity(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)   # Motion
acc = pin.getFrameAcceleration(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
# vel.vector 是 6 维 numpy
```

注意：`getFrameAcceleration` 返回的是“空间加速度”，不是经典加速度；需要经典加速度用 `getFrameClassicalAcceleration`。

---

## 5. 动力学

```python
# 质量矩阵（只填上三角！）
M_upper = pin.crba(model, data, q)             # data.M
M = M_upper + M_upper.T - np.diag(np.diag(M_upper))

# 非线性项 h = C(q,v)v + g(q)
h = pin.nonLinearEffects(model, data, q, v)    # data.nle

# 逆动力学 tau = M qdd + h
tau = pin.rnea(model, data, q, v, qdd)         # data.tau

# 正动力学 qdd = M^{-1}(tau - h)
qdd = pin.aba(model, data, q, v, tau)          # data.ddq

# 一次性算全（FK + CRBA + NLE + 关节雅可比 + CoM + centroidal + 能量）
pin.computeAllTerms(model, data, q, v)
```

`computeAllTerms` 是 WBC 里最方便的入口：一次调用后 `data.M`、`data.nle`、`data.J`、`data.Jcom`、`data.Ag`、`data.hg` 都有了。

**重要**：`data.M` 只填上三角。做 QP/求逆前必须对称化，否则下三角是垃圾值。

---

## 6. CoM 与质心量

```python
# CoM 位置/速度/加速度
com = pin.centerOfMass(model, data, q, v, qdd)     # data.com[0]
vcom = data.vcom[0]
acom = data.acom[0]

# CoM 雅可比 3×nv
Jcom = pin.jacobianCenterOfMass(model, data, q)

# 质心映射 A (6×nv) 与质心动量 hg
Ag = pin.computeCentroidalMap(model, data, q)
hg = pin.computeCentroidalMomentum(model, data, q, v)   # data.hg
dAg = pin.computeCentroidalMapTimeVariation(model, data, q, v)  # data.dAg
```

固定基座不需要质心平衡；以后做轮式底盘/浮动基时用。

---

## 7. 李群运算（浮动基必备）

```python
q_next = pin.integrate(model, q, v * dt)       # q ⊕ v dt
e      = pin.difference(model, q1, q2)         # q1 ⊖ q2
q      = pin.normalize(model, q)               # 四元数归一化

R_err  = pin.log3(R_des @ R.T)                 # 旋转误差（世界系）
M_err  = pin.log6(oMf_des.inverse() * oMf)     # 6D 误差（Motion）
R      = pin.exp3(omega)                       # 旋转向量 → 旋转矩阵
oMf    = pin.exp6(motion)                      # 6D 运动 → SE3
```

固定基座、全 1-DoF 关节时，`q` 就是普通向量，可以直接 `q + v*dt`；但为了以后兼容浮动基，建议统一用 `integrate/difference`。

**坑**：`q1 - q2` 对四元数/连续关节是错的；一律用 `difference`。`q + v*dt` 对四元数不归一化；一律用 `integrate`。

---

## 8. 限位与模型信息

```python
q_min, q_max = model.lowerPositionLimit, model.upperPositionLimit
v_max = model.velocityLimit
tau_max = model.effortLimit

# 力矩限位可能是 inf，要检查
import numpy as np
tau_max = np.where(np.isfinite(tau_max), tau_max, 1000.0)
```

`model.armature` 会进入 `crba` 的 `M` 对角；真机上电机转子惯量不可忽略，建模时要加。

---

## 9. 一个最小 WBC 片段

```python
import numpy as np
import pinocchio as pin

model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()
fid = model.getFrameId("left_ee")

def task_acceleration(q, v, fid, p_des, R_des, v_des=np.zeros(6), a_des=np.zeros(6),
                      Kp=100.0, Kd=20.0):
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.computeJointJacobiansTimeVariation(model, data, q, v)

    oMf = data.oMf[fid]
    e = np.concatenate([
        p_des - oMf.translation,
        pin.log3(R_des @ oMf.rotation.T),
    ])
    J = pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    dJ = pin.getFrameJacobianTimeVariation(
        model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    vel = pin.getFrameVelocity(
        model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector

    a_star = a_des + Kp * e + Kd * (v_des - vel)
    b = a_star - dJ @ v            # 约束 J qdd = b
    return J, b
```

完整的 QP 组装见 [05-implementation.md](05-implementation.md)。

---

## 10. 性能建议

- **复用 Data**：每个线程一个 `Data`，不要每周期 `createData()`。
- **Python 够用**：18 DoF、几十个约束的 QP，Python + OSQP 可以 200–500 Hz。
- **C++ 实时**：需要 1 kHz 或硬实时，用 C++；`Data` 预分配，使用输出参数版本的 API，避免动态分配。
- **避免重复计算**：`computeAllTerms` 一次算完 FK/CRBA/NLE/J/CoM；不要分别调用多次 FK。
- **Eigen 对齐**：C++ 里固定大小 Eigen 类型注意 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` / 对齐分配。
- **QP 规模**：18 变量、~20 约束，OSQP 每次 <1 ms；不用上大求解器。

---

## 11. 常见坑清单

1. `nq ≠ nv`（浮动基）时，`q` 不能直接加减；用 `integrate/difference`。
2. 四元数不归一化会漂移；定期 `normalize`。
3. `data.M` 只填上三角；记得对称化。
4. `ReferenceFrame` 不匹配：`LOCAL_WORLD_ALIGNED` 雅可比配世界系误差；`LOCAL` 配局部系。
5. `getFrameJacobian` 前必须 `computeJointJacobians`；`computeFrameJacobian` 内部会算。
6. `dJ @ v` 不能省；高速时它和 `J qdd` 同量级。
7. `model.getFrameId` 找不到会抛异常；frame 名要和 URDF 一致。
8. URDF 的 fixed joint 变成 frame，不是关节；别在 `model.joints` 里找。
9. `effortLimit` 可能是 `inf`；QP 里要替换成有限值。
10. 重力方向/单位：`model.gravity` 默认 `-9.81`，URDF 单位是米/弧度。
11. 关节顺序：Pinocchio 的 `q` 顺序是 URDF 树遍历顺序，不一定等于 Isaac/ROS 的关节顺序；必须显式映射。
12. `crba` 的 `Convention` 默认 `LOCAL`；需要世界系时显式传 `pin.Convention.WORLD`。

---

## 12. API 速查表

| 需求 | Python API |
|---|---|
| 建模型（固定基座） | `pin.buildModelFromUrdf(path)` |
| 建模型（浮动基） | `pin.buildModelFromUrdf(path, pin.JointModelFreeFlyer())` |
| 建 Data | `model.createData()` |
| 正运动学 | `pin.forwardKinematics(model, data, q, v, a)` |
| frame 更新 | `pin.updateFramePlacements(model, data)` |
| 关节雅可比 | `pin.computeJointJacobians` + `pin.getJointJacobian` |
| frame 雅可比 | `pin.getFrameJacobian` / `pin.computeFrameJacobian` |
| 雅可比导数 | `pin.computeJointJacobiansTimeVariation` + `pin.getFrameJacobianTimeVariation` |
| frame 速度 | `pin.getFrameVelocity` |
| M | `pin.crba` |
| h | `pin.nonLinearEffects` |
| 逆动力学 | `pin.rnea` |
| 正动力学 | `pin.aba` |
| 全量 | `pin.computeAllTerms` |
| CoM | `pin.centerOfMass` |
| CoM 雅可比 | `pin.jacobianCenterOfMass` |
| 质心映射 | `pin.computeCentroidalMap` |
| 质心动量 | `pin.computeCentroidalMomentum` |
| 质心映射导数 | `pin.computeCentroidalMapTimeVariation` |
| 积分 | `pin.integrate` |
| 误差 | `pin.difference` |
| 旋转对数/指数 | `pin.log3` / `pin.exp3` |
| 6D 对数/指数 | `pin.log6` / `pin.exp6` |
| 归一化 | `pin.normalize` |
| 中性配置 | `pin.neutral` |
| 随机配置 | `pin.randomConfiguration` |

---

## 13. 下一步

- 想看完整 WBC QP 代码：跳到 [05-implementation.md](05-implementation.md)。
- 想补理论：看 [01-wbc-principles.md](01-wbc-principles.md) 和 [06-knowledge-and-roadmap.md](06-knowledge-and-roadmap.md)。
