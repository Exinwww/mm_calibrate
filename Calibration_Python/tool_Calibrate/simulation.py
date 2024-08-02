import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
import Tool_calibration
from roboticstoolbox import DHRobot, RevoluteDH

def saveTs(Ts, filename):
    """仿真数据的输出"""
    with open(filename, 'w') as f:
        # print(len(Ts))
        for T in Ts:
            for i in range(4):
                for j in range(4):
                    f.write(str(T[i][j]) + ' ')
            f.write('\n')
    print('Save Ts to ' + filename)

puma = rtb.models.DH.Puma560()
# 末端执行器的变换矩阵
end_effector_transform = SE3.Trans(0, 0, 0.1) * SE3.Rz(np.pi/6)
puma.tool = end_effector_transform
# print(puma.fkine(puma.qz))
print(puma)
# puma.plot(puma.qz, block=True)

# TCP
# 定义目标末端位置
target_position = [0.4, -0.4, 0.2]
# 定义四种姿态
attitudes = [
    [0, -np.pi/2, 0],
    [np.pi/4, np.pi/2, 0],
    [np.pi/2, np.pi/2, 0],
    [-np.pi/4, np.pi/2, 0]
]
Ts = [ ]
for attitude in attitudes:
    T = SE3(target_position) * SE3.RPY(attitude)
    solution = puma.ikine_LM(T)
    fk_result = puma.fkine(solution.q)
    # print(fk_result.t)
    A_j = puma.A(5, solution.q) # 得到末端关节
    # print(A_j)
    Ts.append(np.array(A_j))

t = Tool_calibration.TCP(Ts)
# saveTs(Ts, './data/Tool/tcp.txt')

# TCF
print('################################')
Ts = []
pos1 = [0.4, -0.4, 0.2]
pos2 = [0.5, -0.4, 0.2] # 沿着x轴移动0.1
pos3 = [0.4, -0.4, 0.4] # 沿着z轴移动0.2
attitude = [0, 0, 0]
pos = [pos1, pos2, pos3]
for p in pos:
    T = sm.SE3(p) * sm.SE3.RPY(attitude)
    solution = puma.ikine_LM(T)
    fk_result = puma.fkine(solution.q)
    # print(fk_result)

    A_j = puma.A(5, solution.q)
    Ts.append(np.array(A_j))
t = Tool_calibration.TCF(Ts)
# saveTs(Ts, './data/Tool/tcf.txt')
T_pos3 = sm.SE3(pos3) * sm.SE3.RPY(attitude)
solution = puma.ikine_LM(T_pos3)
print(solution)
