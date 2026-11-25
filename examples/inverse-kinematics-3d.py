import numpy as np
import pinocchio
from numpy.linalg import norm, solve

model = pinocchio.buildSampleModelManipulator()
data = model.createData()

JOINT_ID = 6
# 关节6的目标位姿，旋转是单位矩阵，平移向量
oMdes = pinocchio.SE3(np.eye(3), np.array([1.0, 0.0, 1.0]))

q = pinocchio.neutral(model) # 当前的关节配置
eps = 1e-4 # 收敛的误差 0.1mm
IT_MAX = 1000 #最大的迭代次数
DT = 1e-1 # 每次更新的步长 
damp = 1e-12 # 雅可比的阻尼，防止奇异

it = 0
while True:
    pinocchio.forwardKinematics(model, data, q) # FK 根据当前 q，计算所有关节的世界位姿。
    # 第6号关节当前位姿是：data.oMi[JOINT_ID]
    # actInv 是将目标位姿从世界坐标系转移到当前的关节坐标系中
    # iMd = (oMi)^(-1)* oMd
    iMd = data.oMi[JOINT_ID].actInv(oMdes) 
    # iMd.translation 是: 从当前关节原点指向目标点的向量，并且这个向量使用当前关节坐标系表达。 
    err = iMd.translation 
    if norm(err) < eps: # 由于旋转矩阵不会改变向量长度： 所以虽然 err 表达在局部坐标系中，它的长度仍然是真实的欧氏距离误差。
        success = True
        break
    if it >= IT_MAX:
        success = False
        break
    # 雅可比描述： 每个关节发生一个很小的运动时，末端会怎样运动, v_i = J(q)q'
    J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame j = [6,6]
    J = -J[:3, :]  # linear part of the Jacobian # 这里只拿出来前三行，用来计算先关节速度对末端线速度的影响
    # J[3:6, :]：关节速度对末端角速度的影响 
    v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(3), err))
    q = pinocchio.integrate(model, q, v * DT)
    if not it % 10:
        print(f"{it}: error = {err.T}")
    it += 1

if success:
    print("Convergence achieved!")
else:
    print(
        "\nWarning: the iterative algorithm has not reached convergence to "
        "the desired precision"
    )

print(f"\nresult: {q.flatten().tolist()}")
print(f"\nfinal error: {err.T}")
