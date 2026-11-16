# Pinocchio Python 接口使用手册

> 本文依据 `doc/pinocchio_cheat_sheet.tex` 的接口清单整理，逐项说明作用、参数、输出和典型用法。
>
> - 实测环境：`conda` 环境 `rl` 中的 **Pinocchio 4.0.0**（conda-forge），Python 3.10，NumPy 2.2.6，coal 3.0.3。
> - 仓库源码：`devel` 分支 **4.1.0**，个别接口在 4.1 中有变化，见文末「版本差异与注意事项」。
> - 文中的 `model` 指 `pin.Model`，`data` 指 `model.createData()` 创建的 `pin.Data`。
> - 示例默认加载本仓库的 `models/simple_humanoid.urdf`；部分示例使用 `models/baxter_simple.urdf`（它带有关节限位）。

## 目录

1. [通用约定](#1-通用约定)
2. [空间量与 SE3](#2-空间量与-se3)
3. [Model 与 Data](#3-model-与-data)
4. [模型解析器](#4-模型解析器)
5. [配置空间](#5-配置空间)
6. [运动学与 Frame](#6-运动学与-frame)
7. [雅可比](#7-雅可比)
8. [正/逆动力学](#8-正逆动力学)
9. [质心、动量、能量](#9-质心动量能量)
10. [碰撞检测](#10-碰撞检测)
11. [回归量（辨识）](#11-回归量辨识)
12. [接触与约束动力学](#12-接触与约束动力学)
13. [Cholesky 分解](#13-cholesky-分解)
14. [可视化](#14-可视化)
15. [典型工作流与常见坑](#15-典型工作流与常见坑)
16. [版本差异与注意事项](#16-版本差异与注意事项)
17. [接口速查索引](#17-接口速查索引)

---

## 1. 通用约定

### 1.1 基本导入

```python
import numpy as np
import pinocchio as pin
```

本书不推荐 `from pinocchio.utils import *`，`pinocchio.utils` 里只有少量工具函数（`rpyToMatrix`、`matrixToRpy`、`mprint` 等），按需导入即可。

### 1.2 Model / Data / q / v 的关系

| 概念 | 类型 | 含义 |
|---|---|---|
| `model` | `pin.Model` | 运动学树的**拓扑与参数**：关节、连杆惯量、frame、重力、限位。构建后一般只读。 |
| `data` | `pin.Data` | 算法**工作区/缓存**：每次计算的结果都写在这里。`data = model.createData()`。 |
| `q` | `np.ndarray(nq,)` | 广义位置（configuration）。含四元数/自由飞行关节时 `nq != nv`。 |
| `v` | `np.ndarray(nv,)` | 广义速度（tangent space）。 |
| `a` | `np.ndarray(nv,)` | 广义加速度。 |
| `tau` | `np.ndarray(nv,)` | 广义力/关节力矩。 |

**性能建议**：`data` 只创建一次，在循环里反复使用；每个算法都会原地写入 `data`，不要在每个循环里 `createData()`。

```python
model = pin.buildModelFromUrdf("models/simple_humanoid.urdf")
data = model.createData()

print(model.nq, model.nv, model.njoints, model.nframes)
q = pin.neutral(model)          # (nq,)
v = np.zeros(model.nv)          # (nv,)
```

**不要混用**：一个 `data` 只能配一个 `model`。可以通过 `model.check(data)` 检查一致性。

### 1.3 6 维空间向量的排列顺序

`pin.Motion` 和 `pin.Force` 的 6 维排列都是：

```text
[ linear(3), angular(3) ] = [vx, vy, vz, wx, wy, wz]
```

例如：

```python
m = pin.Motion(np.array([1., 2., 3.]), np.array([4., 5., 6.]))
print(m.linear)    # [1. 2. 3.]
print(m.angular)   # [4. 5. 6.]
```

所有 `6 x nv` 的雅可比矩阵、`6 x 10` 的 body regressor 都遵循同样的顺序：前三行线性部分，后三行角部分。

### 1.4 ReferenceFrame：三种参考系

`pin.ReferenceFrame` 有三个值，凡是带 `reference_frame` 参数的接口都适用：

| 枚举 | 原点 | 坐标轴 | 典型用途 |
|---|---|---|---|
| `ReferenceFrame.LOCAL` | 目标 frame/joint 的原点 | 目标 frame/joint 自身轴向 | 传感器、末端执行器局部量；`computeFrameJacobian` 的默认值 |
| `ReferenceFrame.LOCAL_WORLD_ALIGNED` | 目标 frame/joint 的原点 | 与世界坐标轴对齐 | 在末端点上表达但方向与世界一致，常用于 WBC/MPC |
| `ReferenceFrame.WORLD` | 世界原点 | 世界坐标轴 | 全局量 |

```python
pin.ReferenceFrame.LOCAL
pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
pin.ReferenceFrame.WORLD
```

### 1.5 “先算后用”依赖速查

Pinocchio 很多接口不会隐式帮你算前置量，顺序错了会得到 0 或旧数据。

| 输出字段 | 产生它的接口 | 使用前是否必须先调用 |
|---|---|---|
| `data.oMi` | `pin.forwardKinematics` | 是 |
| `data.oMf` | `pin.updateFramePlacements` / `pin.framesForwardKinematics` | 是 |
| `data.v`, `data.a` | `pin.forwardKinematics(model, data, q, v[, a])` | 是 |
| `data.J` | `pin.computeJointJacobians` | 是（`getJointJacobian` 需要） |
| `data.dJ` | `pin.computeJointJacobiansTimeVariation` | 是（`getJointJacobianTimeVariation` 需要） |
| `data.M` | `pin.crba` / `pin.computeAllTerms` | 是（`cholesky.decompose` 需要） |
| `data.Minv` | `pin.computeMinverse` / `pin.cholesky.computeMinv` | 是 |
| `data.nle` | `pin.nonLinearEffects` / `pin.computeAllTerms` | 否 |
| `data.g` | `pin.computeGeneralizedGravity` | 否 |
| `data.ddq` | `pin.aba` | 否 |
| `data.tau` | `pin.rnea` | 否 |
| `data.dtau_dq`, `data.dtau_dv`, `data.M` | `pin.computeRNEADerivatives` | 否 |
| `data.com`, `data.vcom`, `data.acom` | `pin.centerOfMass` | 否 |
| `data.Jcom` | `pin.jacobianCenterOfMass` | 否 |
| `data.hg`, `data.Ag`, `data.Ig` | `pin.computeCentroidalMomentum` / `pin.computeAllTerms` | 否 |
| `data.dhg` | `pin.computeCentroidalMomentumTimeVariation` | 否 |
| `data.lambda_c` | `pin.forwardDynamics`（约束 Jacobian 版本） | 否 |
| `data.impulse_c`, `data.dq_after` | `pin.impulseDynamics`（约束 Jacobian 版本） | 否 |
| `gd.collisionResults`, `gd.distanceResults` | `pin.computeCollisions` / `pin.computeDistances` | 否 |
| `gd.radius` | `pin.computeBodyRadius` | 否 |

---

## 2. 空间量与 SE3

### 2.1 `pin.SE3`：刚体变换

`SE3` 表示 `aMb`：把 B 坐标系下的量变换到 A 坐标系。

| 接口 | 作用 |
|---|---|
| `pin.SE3(R, p)` | 由旋转矩阵 `R`（3x3）和平移 `p`（3,）构造 |
| `pin.SE3(quat, p)` | 由 `pin.Quaternion` 和平移构造 |
| `pin.SE3(1)` / `pin.SE3.Identity()` | 单位变换 |
| `pin.SE3.Random()` | 随机变换（测试用） |
| `M.rotation` | 3x3 旋转矩阵 |
| `M.translation` | 3 维平移 |
| `M.inverse()` | 逆变换 `bMa` |
| `M1 * M2` | 变换复合 |
| `M.action` | 6x6 伴随矩阵（action matrix） |
| `M.homogeneous` | 4x4 齐次矩阵 |
| `pin.log(M)` | SE3 → 6 维李代数向量 |
| `pin.exp(xi)` | 6 维李代数向量 → SE3 |

```python
R = np.eye(3)
p = np.array([1., 2., 3.])
aMb = pin.SE3(R, p)

print(aMb.translation)       # [1. 2. 3.]
print(aMb.rotation.shape)    # (3, 3)
print(aMb.homogeneous.shape) # (4, 4)
print(aMb.action.shape)      # (6, 6)

bMa = aMb.inverse()
M = pin.SE3.Random()
xi = pin.log(M)              # (6,)
M2 = pin.exp(xi)             # SE3
```

注意：`pin.exp()` 传入 6 维向量返回 `SE3`；`pin.exp(np.zeros(6))` 就是单位变换。

### 2.2 `pin.Motion`：空间速度/加速度

```python
m = pin.Motion(linear, angular)   # 第一个参数是线速度，第二个是角速度
m.linear                          # (3,)
m.angular                         # (3,)
```

变换作用：

```python
aMb = pin.SE3.Random()
v_b = pin.Motion.Random()
v_a = aMb * v_b        # 等价于 aMb.act(v_b)
v_b2 = aMb.actInv(v_a)
```

`Motion` 还支持叉乘 `m1 ^ m2`（或 `m1.cross(m2)`），返回 `Motion`。

### 2.3 `pin.Force`：空间力

```python
f = pin.Force(linear_force, torque)   # 前三个是力，后三个是力矩
f.linear
f.angular
```

坐标变换：

```python
f_a = aMb * f_b        # 等价于 aMb.act(f_b)
```

力与速度可做功率点乘：

```python
power = f.dot(v)
```

### 2.4 `pin.Inertia`：空间惯量

```python
Y = pin.Inertia(mass, com, rotational_inertia)
# mass: float
# com: (3,) 质心位置
# rotational_inertia: (3,3) 关于质心的转动惯量

Y.mass
Y.lever          # 质心位置
Y.inertia        # 3x3 转动惯量
Y.matrix()       # 6x6 空间惯量矩阵
Y.toDynamicParameters()   # 10 维惯性参数向量
```

`pin.Inertia.FromDynamicParameters(phi)` 可以从 10 维参数恢复惯量。惯量可相加：

```python
Y_total = Y1 + Y2
```

### 2.5 `pin.Quaternion` 与 `pin.AngleAxis`

```python
quat = pin.Quaternion(R)          # 从旋转矩阵构造
quat = pin.Quaternion(w, x, y, z) # 从四个标量构造（Eigen 顺序：w 在前）
quat.matrix()                     # 旋转矩阵

aa = pin.AngleAxis(angle, axis)   # axis 为 3 维单位向量
aa.angle
aa.axis
aa.matrix()                       # 旋转矩阵
aa.toRotationMatrix()
```

注意两点：

- 四标量构造函数是 `(w, x, y, z)`，例如 `pin.Quaternion(1, 0, 0, 0)` 是单位四元数。
- 但 `.coeffs()` 和打印结果的顺序是 `(x, y, z, w)`；`pin.SE3ToXYZQUAT` 返回的也是 `[x, y, z, qx, qy, qz, qw]`。两种顺序不要混用。

### 2.6 常用转换

| 接口 | 作用 |
|---|---|
| `pin.SE3ToXYZQUAT(M)` | SE3 → `[x, y, z, qx, qy, qz, qw]`（7 维） |
| `pin.XYZQUATToSE3(vec)` | 7 维 → SE3 |
| `pin.log(M)` / `pin.exp(xi)` | SE3 ↔ 6 维李代数 |
| `pin.Quaternion(R)` | 旋转矩阵 → 四元数 |
| `pin.AngleAxis(angle, axis)` | 轴角 → 旋转 |

```python
vec = pin.SE3ToXYZQUAT(M)    # (7,)
M2 = pin.XYZQUATToSE3(vec)
```

### 2.7 `pin.classicAcceleration`

把空间加速度（Pinocchio 算法内部使用的量）转换为经典加速度：

```python
a_classic = pin.classicAcceleration(v, a)
# 或指定相对变换
a_classic = pin.classicAcceleration(v, a, aMb)
```

返回 `np.ndarray(3,)`。公式为 `a' = a + (0, ω × v_O)`。

---

## 3. Model 与 Data

### 3.1 `pin.Model` 的常用字段

| 字段 | 含义 |
|---|---|
| `model.name` | 模型名 |
| `model.names` | 所有 joint 名字列表（长度 `njoints`） |
| `model.joints` | joint 模型列表 |
| `model.jointPlacements` | 各 joint 相对父 joint 的放置（`SE3`） |
| `model.inertias` | 各 joint 所带 body 的惯量 |
| `model.frames` | 所有 frame（`pin.Frame`） |
| `model.nq` | 配置向量维数 |
| `model.nv` | 速度/加速度/力矩维数 |
| `model.njoints` | joint 数（包含 universe joint 0） |
| `model.nframes` | frame 数 |
| `model.parents` | 父 joint 索引 |
| `model.gravity` | 重力（`pin.Motion`，默认 `[0, 0, -9.81]`） |
| `model.lowerPositionLimit` / `upperPositionLimit` | 位置限位 |
| `model.velocityLimit` | 速度限位 |
| `model.effortLimit` | 力矩限位 |

> 注意：旧版 cheat sheet 写的 `model.placements` 在 4.0 中不存在，正确名字是 `model.jointPlacements`。

### 3.2 查找 joint / frame

```python
jid = model.getJointId("RARM_ELBOW")
fid = model.getFrameId("RARM_LINK7")

model.existJointName("RARM_ELBOW")
model.existFrame("RARM_LINK7")
model.getBodyId("RARM_LINK7")   # 只查 BODY 类型的 frame

# 找不到时 getFrameId 返回 model.nframes，不抛异常，务必检查
if fid == model.nframes:
    raise ValueError("frame not found")
```

### 3.3 手动搭建模型：`addJoint` / `appendBodyToJoint` / `addFrame`

```python
model = pin.Model()

jid = model.addJoint(
    0,                        # parent joint id
    pin.JointModelRX(),       # 绕 X 轴转动
    pin.SE3.Identity(),       # 相对父 joint 的放置
    "joint1",
)

model.appendBodyToJoint(
    jid,
    pin.Inertia(1.0, np.array([0.1, 0.0, 0.0]), np.eye(3)),
    pin.SE3.Identity(),       # body 相对 joint 的放置
)

frame = pin.Frame(
    "tool",
    jid,                      # parent joint
    0,                        # parent frame（0 表示 universe）
    pin.SE3.Identity(),
    pin.FrameType.OP_FRAME,   # 操作 frame
)
fid = model.addFrame(frame)
```

可用的 joint 模型（节选）：

```text
pin.JointModelRX / RY / RZ                  转动关节
pin.JointModelPX / PY / PZ                  移动关节
pin.JointModelRevoluteUnaligned(axis)       任意轴转动
pin.JointModelPrismaticUnaligned(axis)      任意轴移动
pin.JointModelFreeFlyer()                   6 自由度浮动基
pin.JointModelPlanar()                      平面关节
pin.JointModelSpherical()                   球关节
pin.JointModelSphericalZYX()                球关节（ZYX 欧拉角参数化）
pin.JointModelTranslation()                 SE(3) 平移
pin.JointModelComposite(...)                复合关节
```

`addJoint` 还有一个带限位的重载：

```python
jid = model.addJoint(0, pin.JointModelRX(), pin.SE3.Identity(), "j1",
                     max_effort, max_velocity, min_config, max_config)
```

### 3.4 `appendModel` 与 `buildReducedModel`

这两个在 4.0 中是**自由函数**，不是 `Model` 的方法：

```python
# 把 modelB 接到 modelA 的某个 frame 上
model_combined = pin.appendModel(modelA, modelB, frame_in_modelA, aMb)

# 锁定若干 joint，得到降维模型
locked = [model.getJointId("RARM_ELBOW"), model.getJointId("LARM_ELBOW")]
reduced = pin.buildReducedModel(model, locked, pin.neutral(model))
print(reduced.nq, reduced.nv)

# 同时缩减几何模型
reduced_model, reduced_geom = pin.buildReducedModel(
    model, geom_model, locked, pin.neutral(model)
)
```

### 3.5 `pin.Data` 的常用字段

`data = model.createData()` 或 `pin.Data(model)`。

| 字段 | 形状/类型 | 含义与产生者 |
|---|---|---|
| `data.oMi` | list[SE3]，长度 `njoints` | joint 相对世界的放置，`forwardKinematics` |
| `data.oMf` | list[SE3]，长度 `nframes` | frame 相对世界的放置，`updateFramePlacements` |
| `data.v` | list[Motion]，长度 `njoints` | **joint 局部坐标系**下的速度 |
| `data.ov` | list[Motion] | 世界原点表达的速度 |
| `data.a` | list[Motion] | joint 局部加速度 |
| `data.f` | list[Force] | 各 body 的力（rnea/aba 中间量） |
| `data.M` | (nv, nv) | 关节空间惯性矩阵，`crba` |
| `data.Minv` | (nv, nv) | `M` 的逆，`computeMinverse` |
| `data.nle` | (nv,) | 非线性效应 `C(q,v)v + g(q)` |
| `data.g` | (nv,) | 重力项 |
| `data.tau` | (nv,) | `rnea` 的输出 |
| `data.ddq` | (nv,) | `aba` 的输出 |
| `data.J` | (6, nv) | 所有 joint 的雅可比堆叠，`computeJointJacobians` |
| `data.dJ` | (6, nv) | 雅可比时间导数，`computeJointJacobiansTimeVariation` |
| `data.com` | list[Vec3] | 质心，`[0]` 为整机质心 |
| `data.vcom` | list[Vec3] | 质心速度，`[0]` 为整机 |
| `data.acom` | list[Vec3] | 质心加速度，`[0]` 为整机 |
| `data.Jcom` | (3, nv) | 质心雅可比，`jacobianCenterOfMass` |
| `data.mass` | (njoints,) | 各子树质量，`[0]` 为总质量 |
| `data.hg` | `Force` | 质心动量，`computeCentroidalMomentum` |
| `data.Ag` | (6, nv) | 质心动量矩阵 |
| `data.Ig` | `Inertia` | 质心惯量 |
| `data.dhg` | `Force` | 质心动量时间导数 |
| `data.lambda_c` | (nc,) | 接触力，`forwardDynamics` |
| `data.impulse_c` | (nc,) | 接触冲量，`impulseDynamics` |
| `data.dq_after` | (nv,) | 碰撞后速度，`impulseDynamics` |
| `data.dtau_dq` | (nv, nv) | `computeRNEADerivatives` |
| `data.dtau_dv` | (nv, nv) | `computeRNEADerivatives` |
| `data.ddq_dq` / `ddq_dv` / `ddq_dtau` | (nv, nv) | 约束动力学导数 |
| `data.dlambda_dq` / `dlambda_dv` / `dlambda_dtau` | (nc, nv) | 接触力导数 |
| `data.kinetic_energy` / `potential_energy` / `mechanical_energy` | float | 能量接口 |
| `data.staticRegressor` | (3, 4*(njoints-1)) | `computeStaticRegressor` |
| `data.jointTorqueRegressor` | (nv, 10*(njoints-1)) | `computeJointTorqueRegressor` |

> `data.com`、`data.vcom`、`data.acom` 在 4.0 中是 `StdVec_Vec3s`（每个 joint 一项），整机值在索引 `0`。`centerOfMass` 的返回值则是 `np.ndarray(3,)`。

---

## 4. 模型解析器

### 4.1 URDF

```python
# 固定基
model = pin.buildModelFromUrdf("robot.urdf")

# 浮动基
model = pin.buildModelFromUrdf("robot.urdf", pin.JointModelFreeFlyer())

# 指定根关节名字
model = pin.buildModelFromUrdf("robot.urdf", pin.JointModelFreeFlyer(), "root_joint")

# 追加到已有 model
model = pin.buildModelFromUrdf("part2.urdf", model)
```

一次性加载模型 + 碰撞/视觉几何：

```python
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_path,
    package_dirs=mesh_dir,          # 可选，网格所在目录
    root_joint=pin.JointModelFreeFlyer(),   # 可选
    geometry_types=[pin.GeometryType.COLLISION, pin.GeometryType.VISUAL],
)

data, collision_data, visual_data = pin.createDatas(
    model, collision_model, visual_model
)
```

单独加载几何：

```python
geom_model = pin.buildGeomFromUrdf(
    model,
    urdf_path,
    pin.GeometryType.COLLISION,
    package_dirs=["models"],        # 必须用关键字！第 4 个位置参数是 geometry_model
)
```

> 第 4 个位置参数是 `geometry_model`，不是 `package_dirs`。如果写成
> `buildGeomFromUrdf(model, urdf, pin.GeometryType.COLLISION, ["models"])`，
> Pinocchio 会把列表当成 `geometry_model` 并给出警告/错误。

### 4.2 SDF

```python
# 4.0 中 buildModelFromSdf 已标记 deprecated，返回 (model, legacy_constraint_models)
model, constraint_models = pin.buildModelFromSdf(
    "robot.sdf",
    root_link_name="base_link",
)

# 推荐的新接口
model, constraint_models = pin.buildModelAndConstraintsFromSdf(
    "robot.sdf", "base_link"
)
```

`root_link_name` 是 SDF 中作为根 link 的名字，不是关节名。SDF 支持的编译开关是 `BUILD_WITH_SDF_SUPPORT`；conda 包一般已启用。

### 4.3 碰撞对与 SRDF

```python
geom_model.addAllCollisionPairs()      # 同一 parent joint 上的几何不会配对
pin.removeCollisionPairs(model, geom_model, "robot.srdf")
```

`geom_model.collisionPairs` 是 `pin.CollisionPair(index1, index2)` 的列表。

---

## 5. 配置空间

Pinocchio 把配置空间当作李群处理：`q` 在 `nq` 维流形上，`v/a/tau` 在 `nv` 维切空间上。自由飞行关节、四元数关节时 `nq != nv`。

| 接口 | 作用 | 返回/输出 |
|---|---|---|
| `pin.neutral(model)` | 中性配置（通常全 0，四元数为单位四元数） | `(nq,)` |
| `pin.randomConfiguration(model)` | 在模型限位内随机采样 | `(nq,)` |
| `pin.randomConfiguration(model, lb, ub)` | 在给定上下界内采样 | `(nq,)` |
| `pin.normalize(model, q)` | 归一化配置（四元数重新单位化） | `(nq,)` |
| `pin.difference(model, q1, q2)` | q1 到 q2 的切空间差 | `(nv,)` |
| `pin.distance(model, q1, q2)` | 两个配置的标量距离 | `float` |
| `pin.squaredDistance(model, q1, q2)` | 每个 joint 的平方距离 | `(njoints-1,)` |
| `np.sum(pin.squaredDistance(...))` | 平方距离之和（4.0 未暴露 `squaredDistanceSum`） | `float` |
| `pin.interpolate(model, q1, q2, alpha)` | 李群插值 | `(nq,)` |
| `pin.integrate(model, q, v)` | 沿 `v` 积分单位时间 | `(nq,)` |
| `pin.dDifference(model, q1, q2)` | 差分对 q1/q2 的偏导 | 两个 `(nv, nv)` |
| `pin.dDifference(model, q1, q2, arg)` | 只对 `ARG0` 或 `ARG1` 求导 | `(nv, nv)` |
| `pin.dIntegrate(model, q, v)` | 积分对 q/v 的偏导 | 两个 `(nv, nv)` |
| `pin.dIntegrate(model, q, v, arg)` | 只对 `ARG0` 或 `ARG1` 求导 | `(nv, nv)` |

```python
q0 = pin.neutral(model)
q1 = pin.randomConfiguration(model)

dq = pin.difference(model, q0, q1)        # (nv,)
dist = pin.distance(model, q0, q1)        # float
sq = pin.squaredDistance(model, q0, q1)   # (njoints-1,)，distance = sqrt(sum(sq))

q_mid = pin.interpolate(model, q0, q1, 0.5)
q_next = pin.integrate(model, q0, dq)     # q0 沿 dq 走 1 个单位时间

J_diff_q0, J_diff_q1 = pin.dDifference(model, q0, q1)
J_int_q, J_int_v = pin.dIntegrate(model, q0, dq)

J_diff_q0 = pin.dDifference(model, q0, q1, pin.ArgumentPosition.ARG0)
J_int_v = pin.dIntegrate(model, q0, dq, pin.ArgumentPosition.ARG1)
```

要点：

- `difference` 的结果是切空间向量，长度始终是 `nv`，因此 `integrate(model, q, difference(model, q, q2))` 应回到 `q2`（数值误差内）。
- `dDifference` / `dIntegrate` 返回的是**切空间到切空间**的 `(nv, nv)` 矩阵，而不是 `(nv, nq)`。这是李群导数的约定。
- `randomConfiguration(model)` 依赖 `model.lowerPositionLimit/upperPositionLimit`。若关节没有限位（例如本仓库 `models/simple_humanoid.urdf`），可能得到与中性配置相同的采样；需要多样本时请手动给 `lb/ub` 或使用带限位的模型（如 `models/baxter_simple.urdf`）。
- 自由飞行关节的限位中可能包含 `±inf`，直接 `randomConfiguration` 可能产生 `nan`，此时应显式传入有限边界。

---

## 6. 运动学与 Frame

### 6.1 `forwardKinematics`

```python
pin.forwardKinematics(model, data, q)          # 只算 oMi
pin.forwardKinematics(model, data, q, v)       # 同时算 data.v
pin.forwardKinematics(model, data, q, v, a)    # 同时算 data.a
```

- `data.oMi[i]`：joint i 相对世界（universe）的放置。
- `data.v[i]`：joint i **局部坐标系**下的空间速度。
- `data.a[i]`：joint i **局部坐标系**下的空间加速度。
- 需要世界原点表达的速度/加速度时用 `data.ov[i]` / `data.oa[i]`。

### 6.2 Frame 放置

Frame 是挂在 joint 上的点/坐标系（传感器、末端、足端等），和 joint 分开索引。

```python
pin.framesForwardKinematics(model, data, q)
# 等价于：
# pin.forwardKinematics(model, data, q)
# pin.updateFramePlacements(model, data)

print(data.oMf[fid])                 # frame fid 相对世界的 SE3
print(model.frames[fid].name)
print(model.frames[fid].parentJoint) # 该 frame 所属 joint
print(model.frames[fid].placement)   # 相对父 joint 的放置
```

`pin.updateFramePlacements(model, data)` 只根据**当前** `data.oMi` 更新所有 frame 的放置，不会重新做 FK。所以典型顺序是：

```python
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
```

### 6.3 速度/加速度

这些接口要求先调用 `forwardKinematics(model, data, q, v[, a])`：

| 接口 | 作用 |
|---|---|
| `pin.getFrameVelocity(model, data, frame_id, rf)` | frame 的空间速度（`Motion`） |
| `pin.getFrameVelocity(model, data, joint_id, placement, rf)` | 任意“joint + 相对放置”位置的速度 |
| `pin.getFrameAcceleration(model, data, frame_id, rf)` | frame 的空间加速度（`Motion`） |
| `pin.getFrameClassicalAcceleration(model, data, frame_id, rf)` | 经典加速度（`Motion`） |

```python
pin.forwardKinematics(model, data, q, v, a)
pin.updateFramePlacements(model, data)

v_frame = pin.getFrameVelocity(model, data, fid, pin.ReferenceFrame.LOCAL)
a_frame = pin.getFrameAcceleration(model, data, fid, pin.ReferenceFrame.LOCAL)
a_classic = pin.getFrameClassicalAcceleration(model, data, fid, pin.ReferenceFrame.WORLD)
```

### 6.4 运动学导数

```python
pin.computeForwardKinematicsDerivatives(model, data, q, v)
# 或带加速度：
pin.computeForwardKinematicsDerivatives(model, data, q, v, a)
```

之后：

```python
# 速度对 q、v 的偏导
dv_dq, dv_dv = pin.getFrameVelocityDerivatives(
    model, data, fid, pin.ReferenceFrame.LOCAL
)

# 加速度对 q、v、a 的偏导；另外还顺带给出 v 对 q、v 的偏导
dv_dq, da_dq, da_dv, da_da = pin.getFrameAccelerationDerivatives(
    model, data, fid, pin.ReferenceFrame.LOCAL
)

# 对 joint 而非 frame：
dv_dq, dv_dv = pin.getJointVelocityDerivatives(
    model, data, jid, pin.ReferenceFrame.LOCAL
)
dv_dq, da_dq, da_dv, da_da = pin.getJointAccelerationDerivatives(
    model, data, jid, pin.ReferenceFrame.LOCAL
)
```

形状都是 `(6, nv)`。注意 `getFrameAccelerationDerivatives` 返回 4 个矩阵，顺序为 `(v_partial_dq, a_partial_dq, a_partial_dv, a_partial_da)`；其中 `a_partial_da` 等于 `v_partial_dv`。

---

## 7. 雅可比

### 7.1 Joint 雅可比

```python
pin.computeJointJacobians(model, data, q)     # 填 data.J，并做 FK
# 或 data.oMi 已就绪时：
pin.computeJointJacobians(model, data)

J_joint = pin.getJointJacobian(
    model, data, jid, pin.ReferenceFrame.LOCAL
)   # (6, nv)
```

- `data.J` 是 `(6, nv)`，所有 joint 的 motion subspace 堆叠（世界坐标表达）。
- `getJointJacobian` **不会重新计算**，只是从 `data.J` 中提取并变换到指定参考系；所以必须先调用 `computeJointJacobians`。
- 列对应 `v` 的分量。

### 7.2 Frame 雅可比

```python
# 默认参考系是 LOCAL（frame 自身坐标系）
J_frame = pin.computeFrameJacobian(model, data, q, fid)

# 显式指定参考系
J_world = pin.computeFrameJacobian(
    model, data, q, fid, pin.ReferenceFrame.WORLD
)
```

`computeFrameJacobian` 内部会做必要的 FK/joint Jacobian 计算，但**不会完整填充 `data.oMf`**。如果后续还要用 `data.oMf[fid]`，请再调用 `pin.updateFramePlacements(model, data)`。

关系：

```text
v_frame = J_frame @ v
```

其中 `v` 是 `(nv,)` 关节速度，`v_frame` 是 `(6,)` 空间速度。

### 7.3 雅可比的时间导数

```python
pin.computeJointJacobiansTimeVariation(model, data, q, v)
# 填 data.J 和 data.dJ

dJ_joint = pin.getJointJacobianTimeVariation(
    model, data, jid, pin.ReferenceFrame.LOCAL
)   # (6, nv)

# Frame 版本：内部会自己算 computeJointJacobiansTimeVariation 和 updateFramePlacements
dJ_frame = pin.frameJacobianTimeVariation(
    model, data, q, v, fid, pin.ReferenceFrame.LOCAL
)
```

`dJ` 满足：

```text
a_frame = J_frame @ a + dJ_frame @ v
```

（在相应参考系下；经典加速度还需另加向心项。）

---

## 8. 正/逆动力学

### 8.1 基本方程

Pinocchio 使用的刚体动力学方程为：

```text
M(q) ddq + nle(q, v) = tau + J^T f_ext
nle(q, v) = C(q, v) v + g(q)
```

- `M(q)`：关节空间惯性矩阵
- `nle(q, v)`：非线性效应（科氏力/离心力 + 重力）
- `g(q)`：重力项
- `tau`：关节力矩

### 8.2 正动力学

```python
# 质量矩阵
M = pin.crba(model, data, q)          # 返回 (nv, nv)，同时写 data.M

# 质量矩阵的逆
Minv = pin.computeMinverse(model, data, q)   # 返回 (nv, nv)，写 data.Minv

# 正动力学：给定 q, v, tau 求 ddq
ddq = pin.aba(model, data, q, v, tau)         # 返回 (nv,)，写 data.ddq

# 带外力
fext = [pin.Force.Zero() for _ in range(model.njoints)]
ddq = pin.aba(model, data, q, v, tau, fext)

# 外部力表达约定：默认 LOCAL；也可指定 WORLD
ddq = pin.aba(model, data, q, v, tau, fext, pin.Convention.WORLD)
```

注意：

- `computeMinverse(model, data)` 不传 `q` 的重载假设 `aba` 已经运行过（内部依赖 `data` 中的分解结果）。
- `crba` 与 `aba` 都会写入 `data` 的其他中间量；两者不要在同一份 `data` 上“交叉”假设顺序，除非你清楚依赖关系。

### 8.3 逆动力学

```python
# 给定 q, v, a 求 tau
tau = pin.rnea(model, data, q, v, a)          # 返回 (nv,)，写 data.tau

# 带外力
tau = pin.rnea(model, data, q, v, a, fext)

# 只算重力项 g(q)
g = pin.computeGeneralizedGravity(model, data, q)   # 返回 (nv,)，写 data.g

# 非线性效应 nle = C v + g
nle = pin.nonLinearEffects(model, data, q, v)       # 返回 (nv,)，写 data.nle

# 逆动力学导数：dtau/dq, dtau/dv, dtau/da(=M)
dtau_dq, dtau_dv, dtau_da = pin.computeRNEADerivatives(
    model, data, q, v, a
)
# 结果同时写入 data.dtau_dq, data.dtau_dv, data.M

# 带外力
dtau_dq, dtau_dv, dtau_da = pin.computeRNEADerivatives(
    model, data, q, v, a, fext
)
```

`computeRNEADerivatives` 的第三个返回值就是 `M(q)`（即 `dtau/da`），会写入 `data.M`。这在计算 `aba` 的解析导数时很有用。

### 8.4 一次算全：`computeAllTerms`

```python
pin.computeAllTerms(model, data, q, v)
```

等价于依次调用：

```text
forwardKinematics
crba
nonLinearEffects
computeJointJacobians
centerOfMass
jacobianCenterOfMass
ccrba
computeKineticEnergy
computePotentialEnergy
computeGeneralizedGravity
```

结果分散在 `data.M`、`data.nle`、`data.J`、`data.com`、`data.Jcom`、`data.hg`、`data.Ag`、`data.Ig`、`data.kinetic_energy`、`data.potential_energy`、`data.g` 等字段。做 WBC/MPC 时，如果这些量都要用，一次 `computeAllTerms` 比逐个调用更省。

---

## 9. 质心、动量、能量

### 9.1 质量

```python
total_mass = pin.computeTotalMass(model)          # float
total_mass = pin.computeTotalMass(model, data)    # 同时写 data.mass[0]

pin.computeSubtreeMasses(model, data)             # 各 joint 子树质量 → data.mass
```

### 9.2 质心

```python
# 只算质心
com = pin.centerOfMass(model, data, q)            # (3,)

# 同时算质心速度
com = pin.centerOfMass(model, data, q, v)
# data.com[0], data.vcom[0]

# 同时算质心加速度
com = pin.centerOfMass(model, data, q, v, a)
# data.com[0], data.vcom[0], data.acom[0]

# 质心雅可比
Jcom = pin.jacobianCenterOfMass(model, data, q)   # (3, nv)，写 data.Jcom
# 若 data 中已有 FK/质心信息：
Jcom = pin.jacobianCenterOfMass(model, data)
```

`compute_subtree_coms=True`（默认）会额外计算每个子树的质心；不需要时设 `False` 可省时间。

### 9.3 质心动量

```python
# 质心动量 hg（Force：线动量 + 角动量）
hg = pin.computeCentroidalMomentum(model, data, q, v)
# data.hg, data.Ag (6, nv), data.Ig (Inertia)

# 若 FK 已算好
hg = pin.computeCentroidalMomentum(model, data)

# 质心动量 + 时间导数
hg = pin.computeCentroidalMomentumTimeVariation(model, data, q, v, a)
# data.hg, data.dhg
```

相关字段：

- `data.Ag`：质心动量矩阵，`hg = Ag @ v`
- `data.Ig`：质心处的 6x6 空间惯量
- `data.dhg`：`dhg/dt`

### 9.4 能量

```python
Ek = pin.computeKineticEnergy(model, data, q, v)
Ep = pin.computePotentialEnergy(model, data, q)
Em = pin.computeMechanicalEnergy(model, data, q, v)
```

结果同时写入：

```python
data.kinetic_energy
data.potential_energy
data.mechanical_energy
```

`computeMechanicalEnergy` 会顺带算动能和势能。势能依赖 `model.gravity`；做零重力仿真时：

```python
model.gravity.linear = np.zeros(3)
```

---

## 10. 碰撞检测

Pinocchio 4.0 的碰撞后端是 **coal**（原 hpp-fcl）。需要安装 coal（conda-forge 的 `coal` 包），并且构建时启用 `BUILD_WITH_COLLISION_SUPPORT`。

### 10.1 几何模型

```python
geom_model = pin.buildGeomFromUrdf(
    model, urdf_path, pin.GeometryType.COLLISION,
    package_dirs=["models"],
)
geom_model.addAllCollisionPairs()
geom_data = geom_model.createData()
```

也可以手动构造（调试/自定义几何很方便）：

```python
import coal

geom_model = pin.GeometryModel()
jid1 = model.getJointId("LARM_WRIST_R")
jid2 = model.getJointId("RARM_WRIST_R")

geom_model.addGeometryObject(
    pin.GeometryObject("g1", jid1, pin.SE3.Identity(), coal.Box(0.2, 0.2, 0.2))
)
geom_model.addGeometryObject(
    pin.GeometryObject("g2", jid2, pin.SE3.Identity(), coal.Box(0.2, 0.2, 0.2))
)
geom_model.addCollisionPair(pin.CollisionPair(0, 1))
geom_data = geom_model.createData()
```

`pin.GeometryObject` 的完整构造为：

```python
pin.GeometryObject(name, parent_joint, parent_frame, placement,
                   collision_geometry, mesh_path="", mesh_scale=...)
```

### 10.2 更新放置

```python
pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)
# 内部会更新 data.oMi 和 geom_data.oMg

# 若 data.oMi 已经是当前配置：
pin.updateGeometryPlacements(model, data, geom_model, geom_data)
```

`geom_data.oMg[i]` 是第 i 个几何体相对世界的放置。

### 10.3 碰撞与距离

```python
# 所有碰撞对：返回 bool（是否存在碰撞），结果写入 geom_data.collisionResults
has_collision = pin.computeCollisions(model, data, geom_model, geom_data, q)

# 找到第一对就停
has_collision = pin.computeCollisions(
    model, data, geom_model, geom_data, q, True
)

# 单独一对
pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)
is_colliding = pin.computeCollision(geom_model, geom_data, pair_index)

# 所有对的最小距离
pin.computeDistances(model, data, geom_model, geom_data, q)
# 返回的是“最近碰撞对”的下标（不是碰撞对数量！）
closest_pair = pin.computeDistances(geom_model, geom_data)

# 单独一对的距离
res = pin.computeDistance(geom_model, geom_data, pair_index)
print(res.min_distance)
print(res.getNearestPoint1(), res.getNearestPoint2())
```

结果读取：

```python
for i, cp in enumerate(geom_model.collisionPairs):
    cr = geom_data.collisionResults[i]
    dr = geom_data.distanceResults[i]
    print(i, cp.first, cp.second, cr.isCollision(), dr.min_distance)
```

注意：

- `computeDistances` 的返回值是**最近距离碰撞对的下标**（源码中为 `min_index`），没有碰撞对时返回 `len(collisionPairs)`。不要把它当数量用。
- `DistanceResult.nearest_points` 在 4.0 的 Python 绑定中可能报 “No Python class registered for std::array<...>”，请改用 `getNearestPoint1()` / `getNearestPoint2()`。
- 可以通过 `geom_data.activateCollisionPair(i)` / `deactivateCollisionPair(i)` / `deactivateAllCollisionPairs()` 控制哪些对参与计算。

### 10.4 几何半径

```python
pin.computeBodyRadius(model, geom_model, geom_data)
# geom_data.radius[joint_id]：挂在各 joint 上的几何体半径
```

### 10.5 BroadPhase

```python
mgr = pin.BroadPhaseManager_NaiveCollisionManager(model, geom_model, geom_data)
# 也有 DynamicAABBTree / SSaP / SaP / IntervalTree 等实现
mgr.update()
has_collision = pin.computeCollisions(mgr, False)
```

BroadPhase 管理器需要先用 `(model, geom_model, geom_data)` 构造，并保证几何放置已更新。`pin.computeCollisions(mgr, stop_at_first)` 返回 bool。

---

## 11. 回归量（辨识）

回归量用于把动力学/运动学量写成“关于参数的线性形式”，常用于惯性参数辨识、轨迹优化和自适应控制。

### 11.1 运动学回归量

```python
# joint 的放置变化对整棵树的运动学参数的线性映射
Kr_joint = pin.computeJointKinematicRegressor(
    model, data, jid, pin.ReferenceFrame.LOCAL
)   # (6, 6*(njoints-1))

# frame 的放置变化
Kr_frame = pin.computeFrameKinematicRegressor(
    model, data, fid, pin.ReferenceFrame.LOCAL
)   # (6, 6*(njoints-1))
```

### 11.2 静态回归量（质心 ↔ 一阶矩参数）

```python
Sr = pin.computeStaticRegressor(model, data, q)   # (3, 4*(njoints-1))
```

它把每个 body 的一阶矩参数（质量 + 质量×质心，共 4 个/body）映射到整机质心位置。

### 11.3 单刚体回归量

```python
# 由空间速度和加速度得到 6x10 的惯性参数回归量
Y = pin.bodyRegressor(velocity, acceleration)     # velocity/acceleration 为 pin.Motion
# 满足：Ia + v × Iv = Y @ I.toDynamicParameters()

# 刚体挂在 joint 上：需要先跑 RNEA
pin.rnea(model, data, q, v, a)
Y_joint = pin.jointBodyRegressor(model, data, jid)   # (6, 10)

# 刚体挂在 frame 上
Y_frame = pin.frameBodyRegressor(model, data, fid)   # (6, 10)
```

### 11.4 关节力矩回归量

```python
Ytau = pin.computeJointTorqueRegressor(model, data, q, v, a)
# (nv, 10*(njoints-1))，满足 tau = Ytau @ phi
```

其中 `phi` 是所有 body 的动态参数拼接，每个 body 用 `inertia.toDynamicParameters()`（10 维）。可以用 `pin.Inertia.FromDynamicParameters(phi_i)` 还原。

> 维度规律：`njoints-1` 是排除 universe joint 后的可动 body 数。上面例子中 `njoints=30`，因此运动学回归量是 `6 x 174`，力矩回归量是 `29 x 290`，静态回归量是 `3 x 116`。

---

## 12. 接触与约束动力学

Pinocchio 有**两套**接触接口，容易混淆：

1. **旧版 contact 接口**：直接传约束 Jacobian `J` 和 drift，函数为 `forwardDynamics`、`impulseDynamics`、`computeKKTContactDynamicMatrixInverse`。
2. **约束模型接口**：用 `pin.RigidConstraintModel` / `pin.RigidConstraintData` 描述接触，函数为 `constraintDynamics`、`computeConstraintDynamicsDerivatives`、`computeImpulseDynamicsDerivatives`、`getConstraintJacobian`。

### 12.1 旧版：给定约束 Jacobian

约束方程一般写成：

```text
J(q) ddq + drift = 0
```

其中 `drift` 包含 `Jdot v` 等项。

```python
J = ...            # (nc, nv)，例如 pin.getConstraintJacobian 或 pin.computeFrameJacobian
drift = ...        # (nc,)

# 带接触的正动力学
ddq = pin.forwardDynamics(model, data, q, v, tau, J, drift)
# 结果：data.ddq, data.lambda_c

# 若 computeAllTerms 已经调用过
ddq = pin.forwardDynamics(model, data, tau, J, drift)

# 碰撞/冲量动力学
dq_after = pin.impulseDynamics(
    model, data, q, v_before, J,
    restitution_coefficient=0.0,
    damping=0.0,
)
# 结果：data.dq_after, data.impulse_c

# 若 crba 已调用过
dq_after = pin.impulseDynamics(model, data, v_before, J)

# KKT 矩阵的逆：[[M, J^T], [J, 0]]^{-1}
Kinv = pin.computeKKTContactDynamicMatrixInverse(model, data, q, J, damping=0.0)
# 形状 (nv+nc, nv+nc)
```

### 12.2 约束模型接口

`RigidConstraintModel` 描述“哪个 joint/frame 与哪个 joint/frame 之间的约束”：

```python
cm = pin.RigidConstraintModel(
    pin.ContactType.CONTACT_6D,   # 6 维刚性接触；CONTACT_3D 为点接触
    model,
    joint1_id,
    pin.SE3.Identity(),           # 相对 joint1 的放置
    joint2_id,                    # 通常为 0（universe）
    pin.SE3.Identity(),           # 相对 joint2 的放置
    pin.ReferenceFrame.LOCAL,     # 参考系
)
cd = cm.createData()

print(cm.residualSize())          # CONTACT_6D -> 6；CONTACT_3D -> 3
```

准备并求解：

```python
pin.initConstraintDynamics(model, data, [cm], [cd])
# 注意：4.0.0 需要同时传约束数据列表 [cd]
# 4.1/devel 中该接口改为 (model, data, [cm])，见版本差异

ddq = pin.constraintDynamics(model, data, q, v, tau, [cm], [cd])
# 结果：data.ddq, cd.contact_force

# ProximalSettings 可选
settings = pin.ProximalSettings(1e-12, 1e-12, 0, 1)
ddq = pin.constraintDynamics(model, data, q, v, tau, [cm], [cd], settings)
```

接触力在 `cd.contact_force`（`Force` 类型）。每个约束模型有自己的 `cd`；多个约束时传列表。

### 12.3 约束 Jacobian

```python
Jc = pin.getConstraintJacobian(model, data, cm, cd)
# CONTACT_6D -> (6, nv)；CONTACT_3D -> (3, nv)
```

可以把它直接喂给 12.1 的旧版接口。

### 12.4 约束动力学导数

```python
derivs = pin.computeConstraintDynamicsDerivatives(
    model, data, [cm], [cd]
)
# 返回 6 个矩阵：
# ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da
```

调用前必须先运行 `constraintDynamics`。输出的维度：

```text
ddq_dq, ddq_dv, ddq_dtau : (nv, nv)
dlambda_dq, dlambda_dv, dlambda_dtau : (nc, nv)
```

### 12.5 冲量动力学（约束模型版）

```python
dq_after = pin.impulseDynamics(
    model, data, q, v_before, [cm], [cd],
    restitution_coefficient=0.0,
)
# 结果：data.dq_after

# 冲量导数
pin.computeImpulseDynamicsDerivatives(
    model, data, [cm], [cd], 0.0
)
# 无返回值；结果写入：
# data.ddq_dq, data.ddq_dv, data.ddq_dtau
# data.dlambda_dq, data.dlambda_dv, data.dlambda_dtau
```

`computeImpulseDynamicsDerivatives` 要求先调用 `impulseDynamics`。

### 12.6 新旧接口选择建议

- 已经有自定义的约束 Jacobian（例如来自解析约束、QP 控制器）→ 用旧版 `forwardDynamics` / `impulseDynamics`。
- 接触是标准的刚性接触/点接触，且需要接触力、导数、与约束求解器配合 → 用 `RigidConstraintModel` 接口。
- 4.1 引入了更新的 `ConstraintModel`/`ConstraintData` 体系和 ADMM/PGS 求解器，详见 `examples/admm-constraint-solver.py`、`examples/g1-constraint-simulation.py`。

---

## 13. Cholesky 分解

基于 `data.M`（先调用 `pin.crba`）：

```python
M = pin.crba(model, data, q)                 # 必须先填 data.M

pin.cholesky.decompose(model, data)          # 分解 data.M

y = np.random.randn(model.nv)
x = pin.cholesky.solve(model, data, y)       # 解 M x = y

Minv = pin.cholesky.computeMinv(model, data) # M^{-1}
```

要点：

- `decompose` 使用 `data.M` 的**上三角部分**，因此必须先 `crba`。
- `solve` / `computeMinv` 依赖 `decompose` 的结果。
- 与 `pin.computeMinverse` 的区别：后者基于 ABA 扩展；Cholesky 是通用稠密分解。两者结果应一致，性能特性不同。

---

## 14. 可视化

Pinocchio 本身不带渲染器，`pinocchio.visualize` 提供对 Meshcat、Gepetto、Panda3D、RViz、Viser 的适配。注意 `pin.visualize` 需要显式导入：

```python
from pinocchio.visualize import MeshcatVisualizer
# 或者 import pinocchio.visualize
```

### 14.1 Meshcat 基本用法

依赖：`pip install meshcat` 或 conda 的 `meshcat-python`。

```python
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_path, package_dirs=mesh_dir
)

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)      # 启动 meshcat server 并打开浏览器
viz.loadViewerModel()          # 把机器人模型加载进场景

viz.display(q)                 # 按 q 显示
viz.displayCollisions(True)    # 显示碰撞体
viz.displayVisuals(True)       # 显示视觉体
```

`initViewer` 的签名是：

```python
initViewer(viewer=None, open=False, loadModel=False, zmq_url=None)
```

### 14.2 添加基本几何体

```python
import meshcat.geometry as mg

name = "target"
viz.viewer[name].set_object(
    mg.Sphere(0.05),
    mg.MeshLambertMaterial(color=0xff0000),
)

viz.viewer["box"].set_object(
    mg.Box([0.1, 0.2, 0.3]),
    mg.MeshLambertMaterial(color=0x00ff00),
)
```

### 14.3 设置位姿

Meshcat 的 `set_transform` 接受 4x4 齐次矩阵。Pinocchio 的 `SE3.homogeneous` 正好是 4x4：

```python
M = pin.SE3(R, p)
viz.viewer[name].set_transform(M.homogeneous)

# 也可以把 SE3 转成 meshcat 常用的 (rotation, translation)：
# 但本仓库 4.0 的实现使用 4x4，直接传 homogeneous 最稳妥。
```

需要每帧刷新时：

```python
viz.display(q)
```

### 14.4 其他 viewer

```python
from pinocchio.visualize import GepettoVisualizer, Panda3dVisualizer, RVizVisualizer
```

接口与 `MeshcatVisualizer` 基本一致（`initViewer` / `loadViewerModel` / `display`），但依赖各自的运行时（gepetto-viewer、panda3d、ROS）。

---

## 15. 典型工作流与常见坑

### 15.1 读取模型并做一次完整计算

```python
import numpy as np
import pinocchio as pin

urdf = "models/simple_humanoid.urdf"
model = pin.buildModelFromUrdf(urdf)
data = model.createData()

q = pin.neutral(model)
v = np.zeros(model.nv)
a = np.zeros(model.nv)

# 运动学
pin.forwardKinematics(model, data, q, v, a)
pin.updateFramePlacements(model, data)

# 雅可比
pin.computeJointJacobians(model, data, q)
fid = model.getFrameId("RARM_LINK7")
J = pin.computeFrameJacobian(model, data, q, fid, pin.ReferenceFrame.LOCAL)

# 动力学
M = pin.crba(model, data, q)
nle = pin.nonLinearEffects(model, data, q, v)
tau = pin.rnea(model, data, q, v, a)
ddq = pin.aba(model, data, q, v, tau)

# 质心/动量
com = pin.centerOfMass(model, data, q, v)
Jcom = pin.jacobianCenterOfMass(model, data)
hg = pin.computeCentroidalMomentum(model, data, q, v)

# 能量
Ek = pin.computeKineticEnergy(model, data, q, v)
Ep = pin.computePotentialEnergy(model, data, q)
```

### 15.2 逆动力学控制/力矩计算

```python
# 期望加速度 a_des，前馈 + 反馈
tau = pin.rnea(model, data, q, v, a_des)

# 若需要重力补偿
tau_g = pin.computeGeneralizedGravity(model, data, q)
```

注意 `rnea` 的 `a` 是**广义加速度**，不是末端加速度。

### 15.3 末端速度/雅可比

```python
pin.forwardKinematics(model, data, q, v)
pin.updateFramePlacements(model, data)

J = pin.computeFrameJacobian(model, data, q, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
v_frame = J @ v
# 或
v_frame = pin.getFrameVelocity(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
```

### 15.4 碰撞检查

```python
geom_model = pin.buildGeomFromUrdf(
    model, urdf, pin.GeometryType.COLLISION, package_dirs=["models"]
)
geom_model.addAllCollisionPairs()
geom_data = geom_model.createData()

pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)
collision = pin.computeCollisions(model, data, geom_model, geom_data, q)
pin.computeDistances(model, data, geom_model, geom_data, q)

for i in range(len(geom_model.collisionPairs)):
    print(i, geom_data.collisionResults[i].isCollision(),
          geom_data.distanceResults[i].min_distance)
```

### 15.5 常见坑清单

1. **`data` 复用**：每个模型只创建一个 `data` 并在循环里复用；不要跨模型共享。
2. **`data.oMf` 不会自动更新**：`forwardKinematics` 只更新 `data.oMi`；frame 放置必须 `updateFramePlacements` 或 `framesForwardKinematics`。
3. **`getJointJacobian` 之前必须先 `computeJointJacobians`**：否则 `data.J` 是旧值或零。
4. **`computeFrameJacobian` 默认 `LOCAL`**：想要世界坐标请显式传 `pin.ReferenceFrame.WORLD`。
5. **frame 与 joint 索引不同**：`getFrameId` / `getJointId` 分开用；找不到时 `getFrameId` 返回 `model.nframes`，不会抛异常。
6. **`nq != nv`**：自由飞行/四元数关节下，`q` 用 `nq`，`v/a/tau` 用 `nv`；`difference`/`integrate` 负责两者转换。
7. **`dDifference` / `dIntegrate` 返回 `(nv, nv)`**：它们是切空间导数，不是 `(nv, nq)`。
8. **`computeDistances` 返回最近碰撞对下标**：不是距离，也不是碰撞对数量。
9. **`DistanceResult.nearest_points` 可能不可用**：用 `getNearestPoint1()` / `getNearestPoint2()`。
10. **`buildGeomFromUrdf` 第 4 个位置参数是 `geometry_model`**：`package_dirs` 请用关键字。
11. **旧版 `forwardDynamics` / `impulseDynamics` 需要 `computeAllTerms` 或 `crba` 作为前置**（取决于重载）；带 `q` 的重载会自己算。
12. **`computeMinverse(model, data)` 不带 `q` 的重载依赖 `aba` 的结果**；不带 `q` 的 `getFrameVelocity` 等则依赖之前 `forwardKinematics` 的结果。
13. **修改 `data.M` 等字段**：用 `data.M[:] = ...`，不要直接重新赋值 `data.M = ...`（会替换 Python 属性而不是写底层缓冲）。
14. **`model.gravity`**：默认 `[0, 0, -9.81]`；零重力要显式置零。
15. **`pin.visualize`**：`import pinocchio as pin` 后不一定有 `pin.visualize`，需要 `import pinocchio.visualize` 或 `from pinocchio.visualize import MeshcatVisualizer`。

---

## 16. 版本差异与注意事项

本文以 `rl` 环境中的 **Pinocchio 4.0.0** 实测为准。仓库 `devel` 是 **4.1.0**，以下差异需要注意：

### 16.1 `initConstraintDynamics` 参数

- **4.0.0**（当前 `rl` 环境）：
  ```python
  pin.initConstraintDynamics(model, data, contact_models, contact_datas)
  ```
- **4.1 / devel**：绑定改为
  ```python
  pin.initConstraintDynamics(model, data, contact_models)
  ```

以运行环境为准，可用 `help(pin.initConstraintDynamics)` 查看。

### 16.2 SDF 接口

- `buildModelFromSdf` 在 4.0 中已 deprecated，并返回 `(model, legacy_constraint_models)`。
- 推荐 `buildModelAndConstraintsFromSdf(filename, root_link_name)`（新约束 API）或 `buildModelAndLegacyConstraintsFromSdf`（旧约束 API）。

### 16.3 `squareDistance` → `squaredDistance`

- cheat sheet 中写的 `pin.squareDistance` 在当前版本不存在。
- 正确的是 `pin.squaredDistance(model, q1, q2)`，返回**每个 joint 的平方距离向量**（长度 `njoints-1`）。
- 标量距离用 `pin.distance(model, q1, q2)`；平方和用 `np.sum(pin.squaredDistance(model, q1, q2))`（4.0 的 Python 绑定没有暴露 `squaredDistanceSum`）。

### 16.4 `model.placements`

- cheat sheet 中写的 `model.placements` 在 4.0 中不存在，正确字段是 `model.jointPlacements`。

### 16.5 4.1 的新约束 API

4.1 增加了 `PointContactConstraintModel`、`FrameAnchorConstraintModel`、`JointLimitConstraintModel`、`DelassusOperator`、`ADMMConstraintSolver`、`PGSConstraintSolver` 等新接口，和本文第 12 章的 `RigidConstraintModel` 是并行的两套体系。学习示例可看：

```text
examples/admm-constraint-solver.py
examples/g1-constraint-simulation.py
examples/constant-length-constraint-kinematics.py
examples/constant-length-constraint-3d.py
```

### 16.6 编译功能开关

Python 包是否包含某些接口取决于编译选项：

| CMake 选项 | 影响 |
|---|---|
| `BUILD_PYTHON_INTERFACE` | 是否生成 Python 绑定 |
| `BUILD_WITH_URDF_SUPPORT` | `buildModelFromUrdf` |
| `BUILD_WITH_SDF_SUPPORT` | `buildModelFromSdf` |
| `BUILD_WITH_COLLISION_SUPPORT` | 碰撞接口（依赖 coal） |
| `BUILD_WITH_CASADI_SUPPORT` | CasADi 相关 |
| `BUILD_WITH_AUTODIFF_SUPPORT` | CppAD 相关 |

conda-forge 的 `pinocchio` 包通常开启了 URDF、碰撞和 Python 绑定；SDF/CasADi/CppAD 不一定开启。可以用 `hasattr(pin, "...")` 检查。

---

## 17. 接口速查索引

| 模块 | 接口 | 一句话作用 |
|---|---|---|
| 空间量 | `pin.SE3`, `pin.Motion`, `pin.Force`, `pin.Inertia` | 刚体变换、空间速度、空间力、空间惯量 |
| 空间量 | `pin.Quaternion`, `pin.AngleAxis` | 旋转表示 |
| 空间量 | `pin.log`, `pin.exp` | SE3 ↔ 6 维李代数 |
| 空间量 | `pin.SE3ToXYZQUAT`, `pin.XYZQUATToSE3` | SE3 ↔ 7 维位姿向量 |
| 空间量 | `pin.classicAcceleration` | 空间加速度 → 经典加速度 |
| 模型 | `pin.Model`, `model.createData()` | 运动学树与工作区 |
| 模型 | `model.addJoint`, `model.appendBodyToJoint`, `model.addFrame` | 手动搭建模型 |
| 模型 | `pin.appendModel`, `pin.buildReducedModel` | 拼接/降维模型 |
| 解析 | `pin.buildModelFromUrdf`, `pin.buildModelsFromUrdf` | 加载 URDF |
| 解析 | `pin.buildModelFromSdf`, `pin.buildModelAndConstraintsFromSdf` | 加载 SDF |
| 解析 | `pin.buildGeomFromUrdf`, `pin.buildGeomFromSdf` | 加载碰撞/视觉几何 |
| 配置 | `pin.neutral`, `pin.randomConfiguration`, `pin.normalize` | 生成/归一化配置 |
| 配置 | `pin.difference`, `pin.distance`, `pin.squaredDistance` | 配置间距离 |
| 配置 | `pin.interpolate`, `pin.integrate` | 李群插值/积分 |
| 配置 | `pin.dDifference`, `pin.dIntegrate` | 配置运算的导数 |
| 运动学 | `pin.forwardKinematics` | 前向运动学（joint 放置/速度/加速度） |
| 运动学 | `pin.framesForwardKinematics`, `pin.updateFramePlacements` | 更新 frame 放置 |
| 运动学 | `pin.getFrameVelocity`, `pin.getFrameAcceleration`, `pin.getFrameClassicalAcceleration` | frame 速度/加速度 |
| 运动学 | `pin.computeForwardKinematicsDerivatives` | 运动学导数缓存 |
| 运动学 | `pin.getFrameVelocityDerivatives`, `pin.getFrameAccelerationDerivatives` | frame 速度/加速度导数 |
| 运动学 | `pin.getJointVelocityDerivatives`, `pin.getJointAccelerationDerivatives` | joint 速度/加速度导数 |
| 雅可比 | `pin.computeJointJacobians`, `pin.getJointJacobian` | joint 雅可比 |
| 雅可比 | `pin.computeFrameJacobian` | frame 雅可比 |
| 雅可比 | `pin.computeJointJacobiansTimeVariation`, `pin.getJointJacobianTimeVariation` | 雅可比时间导数 |
| 雅可比 | `pin.frameJacobianTimeVariation` | frame 雅可比时间导数 |
| 动力学 | `pin.crba` | 质量矩阵 `M` |
| 动力学 | `pin.computeMinverse` | `M^{-1}`（ABA 扩展） |
| 动力学 | `pin.aba` | 正动力学 `ddq` |
| 动力学 | `pin.rnea` | 逆动力学 `tau` |
| 动力学 | `pin.computeGeneralizedGravity` | 重力项 `g` |
| 动力学 | `pin.computeRNEADerivatives` | 逆动力学导数 |
| 动力学 | `pin.nonLinearEffects` | 非线性效应 `nle` |
| 动力学 | `pin.computeAllTerms` | 一次算齐常用动力学量 |
| 质心 | `pin.computeTotalMass`, `pin.computeSubtreeMasses` | 质量 |
| 质心 | `pin.centerOfMass`, `pin.jacobianCenterOfMass` | 质心与质心雅可比 |
| 质心 | `pin.computeCentroidalMomentum`, `pin.computeCentroidalMomentumTimeVariation` | 质心动量及其导数 |
| 能量 | `pin.computeKineticEnergy`, `pin.computePotentialEnergy`, `pin.computeMechanicalEnergy` | 动能/势能/机械能 |
| 碰撞 | `pin.updateGeometryPlacements` | 更新几何放置 |
| 碰撞 | `pin.computeCollisions`, `pin.computeCollision` | 碰撞检测 |
| 碰撞 | `pin.computeDistance`, `pin.computeDistances` | 距离 |
| 碰撞 | `pin.computeBodyRadius` | 关节几何半径 |
| 碰撞 | `pin.BroadPhaseManager_*` | BroadPhase 管理 |
| 回归量 | `pin.computeJointKinematicRegressor`, `pin.computeFrameKinematicRegressor` | 运动学回归量 |
| 回归量 | `pin.computeStaticRegressor` | 静态/质心回归量 |
| 回归量 | `pin.bodyRegressor`, `pin.jointBodyRegressor`, `pin.frameBodyRegressor` | 刚体惯性参数回归量 |
| 回归量 | `pin.computeJointTorqueRegressor` | 力矩回归量 |
| 接触 | `pin.forwardDynamics`, `pin.impulseDynamics`, `pin.computeKKTContactDynamicMatrixInverse` | 旧版约束 Jacobian 接口 |
| 接触 | `pin.RigidConstraintModel`, `pin.RigidConstraintData` | 约束模型 |
| 接触 | `pin.initConstraintDynamics`, `pin.constraintDynamics` | 约束正动力学 |
| 接触 | `pin.computeConstraintDynamicsDerivatives`, `pin.computeImpulseDynamicsDerivatives` | 约束动力学导数 |
| 接触 | `pin.getConstraintJacobian` | 约束雅可比 |
| 线性代数 | `pin.cholesky.decompose`, `pin.cholesky.solve`, `pin.cholesky.computeMinv` | 质量矩阵 Cholesky |
| 可视化 | `MeshcatVisualizer` / `GepettoVisualizer` / `Panda3dVisualizer` | 模型显示 |

---

## 参考

- 仓库中的速查表：`doc/pinocchio_cheat_sheet.tex`
- 官方示例：`examples/*.py`、`examples/*.cpp`
- 教程：`doc/b-examples/`（模型、显示、碰撞、逆运动学、降维模型）
- 练习：`doc/d-practical-exercises/`
- 构建与开发：`development/build.md`
