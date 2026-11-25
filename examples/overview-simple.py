import pinocchio

model = pinocchio.buildSampleModelManipulator()
data = model.createData()

q = pinocchio.neutral(model)
v = pinocchio.utils.zero(model.nv)
a = pinocchio.utils.zero(model.nv)

# 基座 → 末端：计算各连杆的速度、加速度、惯性力
# 末端 → 基座：累加子树受力，并投影成关节力矩
tau = pinocchio.rnea(model, data, q, v, a)
print("tau = ", tau.T)
