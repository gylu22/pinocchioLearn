# 三维位置逆运动学示例讲解

本文对应 [`inverse-kinematics-3d.py`](inverse-kinematics-3d.py)，说明如何使用正运动学、位置误差、关节雅可比和阻尼伪逆，迭代求解机械臂的三维位置逆运动学。

这个示例只控制第 6 号关节原点的位置，不控制它的姿态。理解本例不要求先完整掌握李群和李代数；这里主要使用普通三维向量、坐标系变换和线性近似。

## 问题定义

正运动学根据关节配置计算末端位置：

\[
q \longmapsto p(q)
\]

逆运动学则已知期望位置 \(p_d\)，寻找满足下面条件的关节配置：

\[
p(q)=p_d
\]

机械臂的正运动学通常是非线性的，因此示例没有直接求解析解，而是使用迭代方法：

```text
当前关节配置 q
       ↓
正运动学计算当前位置
       ↓
计算当前位置到目标位置的误差
       ↓
使用雅可比求关节调整方向
       ↓
更新 q 并重复计算
```

## 创建模型和计算数据

```python
model = pinocchio.buildSampleModelManipulator()
data = model.createData()
```

`model` 保存机器人不随计算改变的信息，例如关节类型、父子关系和关节安装位姿。`data` 保存算法运行时的结果，例如每个关节相对世界坐标系的位姿。

这个示例模型是一个 6 自由度固定基机械臂，因此通常有：

```text
model.nq = 6
model.nv = 6
```

## 指定受控关节和目标位置

```python
JOINT_ID = 6
oMdes = pinocchio.SE3(np.eye(3), np.array([1.0, 0.0, 1.0]))
```

`JOINT_ID` 是 Pinocchio 模型中的关节编号，不是 `q` 的数组下标。这里控制第 6 号关节坐标系的原点。

`oMdes` 是目标 SE(3) 位姿：

\[
{}^oM_d=
\begin{bmatrix}
I_{3\times3} & [1,0,1]^T\\
0 & 1
\end{bmatrix}
\]

其中：

- `np.eye(3)` 是目标旋转矩阵。
- `[1, 0, 1]` 是目标在世界坐标系中的位置。

本例后续只读取 `oMdes` 的位置，因此目标旋转不会被控制。目标旋转即使换成其他值，也不会改变这个位置任务的定义。

## 初始化求解参数

```python
q = pinocchio.neutral(model)
eps = 1e-4
IT_MAX = 1000
DT = 1e-1
damp = 1e-12
```

各参数含义如下：

| 参数 | 含义 |
|---|---|
| `q` | 当前关节配置，初值为中性配置 |
| `eps` | 收敛阈值；模型使用米时，`1e-4` 为 0.1 mm |
| `IT_MAX` | 最大迭代次数 |
| `DT` | 每次采用关节调整量的比例，即搜索步长 |
| `damp` | 阻尼项，用于改善奇异位置附近的数值稳定性 |

`DT` 不是动力学仿真的物理时间。它只决定每轮沿求解方向前进多少。

## 正运动学计算当前位姿

```python
pinocchio.forwardKinematics(model, data, q)
```

该函数根据当前 `q` 更新所有关节的世界位姿。第 6 号关节的当前位姿保存在：

```python
data.oMi[JOINT_ID]
```

可以写成：

\[
{}^oM_i=
\begin{bmatrix}
{}^oR_i & {}^op_i\\
0 & 1
\end{bmatrix}
\]

其中 \({}^oR_i\) 是当前关节的世界朝向，\({}^op_i\) 是当前关节原点的世界位置。

## 将目标转换到当前关节坐标系

```python
iMd = data.oMi[JOINT_ID].actInv(oMdes)
```

这等价于：

\[
{}^iM_d=({}^oM_i)^{-1}{}^oM_d
\]

它不是改变目标的物理位置，而是改用当前关节坐标系描述同一个目标。

设当前位姿为 \((R_i,p_i)\)，目标世界位置为 \(p_d\)，那么目标在当前关节坐标系中的位置为：

\[
{}^ip_d=R_i^T(p_d-p_i)
\]

代码将这个局部坐标向量作为位置误差：

```python
err = iMd.translation
```

因此 `err` 的含义是：

> 从当前关节原点指向目标点的向量，并且使用当前关节坐标轴表达。

如果当前关节已经到达目标位置，那么 `err` 为零。

### 为什么误差使用局部坐标系

本例中的 `computeJointJacobian` 返回局部关节坐标系下的雅可比。误差和雅可比必须使用相同的表达坐标系，否则不能直接代入同一个线性方程。

虽然 `err` 使用局部坐标系表达，但旋转不会改变向量长度：

\[
\|R_i^T(p_d-p_i)\|=\|p_d-p_i\|
\]

因此下面的判断仍然代表真实的欧氏位置误差：

```python
if norm(err) < eps:
    success = True
    break
```

## 使用雅可比描述局部运动

```python
J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)
```

关节雅可比建立广义速度和关节空间速度之间的局部线性关系：

\[
v_i=J_i(q)v
\]

Pinocchio 的 6 维空间运动向量按“线速度、角速度”排列，因此：

```text
J[0:3, :]  关节速度对受控关节线速度的影响
J[3:6, :]  关节速度对受控关节角速度的影响
```

本例只控制位置，所以只需要线速度部分：

```python
J_position = J[:3, :]
```

对 6 自由度机械臂，完整雅可比为 \(6\times6\)，位置雅可比为 \(3\times6\)。对 7 自由度机械臂，位置雅可比则为 \(3\times7\)。

位置雅可比的第 \(j\) 列表示：第 \(j\) 个广义速度取单位值时，受控关节原点产生的局部线速度。

## 位置误差雅可比的负号

代码写成：

```python
J = -J[:3, :]
```

位置误差定义为“目标减去当前值”：

\[
e=p_d-p(q)
\]

当前末端位置的小变化近似为：

\[
\Delta p\approx J_p\Delta q
\]

目标位置保持不变，所以误差变化近似为：

\[
\Delta e\approx-\Delta p\approx-J_p\Delta q
\]

因此误差雅可比近似为：

\[
J_e=-J_p
\]

代码执行 `J = -J[:3, :]` 后，变量 `J` 表示的是误差雅可比，而不再只是原始的位置雅可比。

局部线性化可以写成：

\[
e(q+\Delta q)\approx e(q)+J_e\Delta q
\]

为了让下一轮误差接近零，希望：

\[
J_e\Delta q=-e
\]

## 使用阻尼伪逆计算关节调整方向

```python
v = -J.T.dot(
    solve(J.dot(J.T) + damp * np.eye(3), err)
)
```

对应公式为：

\[
v=-J^T(JJ^T+\lambda I)^{-1}e
\]

其中：

\[
J^\#_\lambda=J^T(JJ^T+\lambda I)^{-1}
\]

称为阻尼伪逆，因此也可以写成：

\[
v=-J^\#_\lambda e
\]

这里的 `v` 是当前 IK 迭代建议采用的关节空间搜索方向，不一定代表机器人最终执行的物理速度。

### 为什么不能直接使用普通逆矩阵

位置雅可比通常不是方阵。例如 6 自由度机械臂的位置雅可比为：

\[
J_p\in\mathbb{R}^{3\times6}
\]

所以不存在普通的 \(J_p^{-1}\)。即使雅可比是方阵，也可能在机械臂伸直等姿态附近变得奇异。

阻尼项：

\[
\lambda I
\]

可以避免奇异附近的关节调整量过大。示例中的 `1e-12` 很小，主要防止严格的数值奇异；工程应用通常需要根据任务尺度调节阻尼。

### `solve` 的作用

代码没有显式计算矩阵逆，而是先求解：

\[
(JJ^T+\lambda I)y=e
\]

然后计算：

\[
v=-J^Ty
\]

使用 `solve(A, b)` 通常比显式计算 `inv(A) @ b` 更稳定。

## 在构型空间中更新关节配置

```python
q = pinocchio.integrate(model, q, v * DT)
```

对于全部由普通旋转关节组成的固定基机械臂，可以近似理解为：

\[
q_{k+1}\approx q_k+DT\,v_k
\]

Pinocchio 使用 `integrate`，是为了同时正确支持四元数球形关节、浮动基座和平面关节等非欧式构型。这些模型的 `q` 和 `v` 可能维度不同，不能直接执行 `q += v * DT`。

`DT=0.1` 表示每轮只采用当前建议调整量的一部分。步长过大可能振荡或越过目标，步长过小则会降低收敛速度。

## 完整迭代公式

每轮计算可以概括为：

\[
\begin{aligned}
M_i(q_k) &= FK(q_k)\\
e_k &= R_i^T(p_d-p_i)\\
J_{e,k} &\approx -J_{p,k}\\
v_k &= -J_{e,k}^T
\left(J_{e,k}J_{e,k}^T+\lambda I\right)^{-1}e_k\\
q_{k+1} &= \operatorname{integrate}(q_k,DT\,v_k)
\end{aligned}
\]

迭代在满足下面任一条件时结束：

```text
位置误差小于 eps：求解成功
迭代次数达到 IT_MAX：求解失败
```

## 和李群李代数的关系

这个三维位置版本不需要使用完整的李代数误差。

代码中的 `SE3` 表示一个刚体位姿：

\[
M=(R,p),\qquad R\in SO(3),\quad p\in\mathbb{R}^3
\]

这里主要使用了两项 SE(3) 运算：

1. 通过 `forwardKinematics` 组合各级刚体位姿。
2. 通过 `current.inverse() * target` 将目标转换到当前关节坐标系。

误差本身仍然只是普通的三维平移向量，没有使用 `log6`、`exp6` 或 `Jlog6`。

可以先把李群直观地理解为：

> SE(3) 是一套正确组合刚体旋转、平移和坐标系转换的规则。

完整 6D 逆运动学还需要同时处理旋转误差。此时不能直接用两个旋转矩阵相减，而要使用 `log6` 将相对位姿转换成 6 维切空间误差，并使用 `Jlog6` 计算该误差的导数。

## 示例的简化和局限

这个示例适合解释基本算法，但不是完整的工程 IK 求解器：

- 它控制的是关节坐标系原点，不一定是机器人实际 TCP。
- 它只控制位置，不控制末端方向。
- 它没有处理关节位置和速度限制。
- 它没有处理碰撞或自碰撞。
- 它不保证不可达目标能够收敛。
- 它没有使用冗余自由度完成避障、远离关节限位等次级任务。
- 它用 `-J[:3, :]` 近似局部误差雅可比，忽略了局部坐标系旋转产生的耦合项；该项在接近目标时趋近于零，因此简单迭代通常仍能收敛。

对于 7 自由度机械臂，三维位置任务只有 3 个约束，却有 7 个关节变量，所以通常存在很多可行关节姿态。阻尼伪逆会给出一种较小范数的局部调整方向，但不会主动选择最适合避障或远离关节限位的解。

## 推荐的调试输出

学习时可以增加以下输出，观察每一步的维度和物理含义：

```python
current = data.oMi[JOINT_ID]

print("current world position:", current.translation)
print("target world position:", oMdes.translation)
print("local error:", err)
print("error norm:", norm(err))
print("Jacobian shape:", J.shape)
print("joint update direction:", v)
```

也可以验证局部误差与世界误差的关系：

```python
error_world = oMdes.translation - current.translation
error_local = current.rotation.T @ error_world

print("error from iMd:", iMd.translation)
print("error transformed manually:", error_local)
```

两者应当在数值误差范围内一致。
