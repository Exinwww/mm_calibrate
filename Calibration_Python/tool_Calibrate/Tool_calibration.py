import numpy as np

def HomogeneousMtr2Rt(T):
    """
    从齐次变换矩阵提取旋转矩阵和平移向量
    :param: T :齐次变换矩阵
    return: R, t
    """
    R = T[:3, :3]
    t = T[:3, 3]

    return R, t

def TCP(Ts):
    """
    TCP标定，用于标定位置矢量
    :param: Ts :齐次变换矩阵list
    """
    Rs = []
    ts = []
    for T in Ts:
        R, t = HomogeneousMtr2Rt(T)
        Rs.append(R)
        ts.append(t)
    
    # 若直接六点标定，则无需双层循环
    left = []
    right = []
    for i in range(len(Rs)-1):
        for j in range(i+1, len(Rs)):
            left.append(Rs[i] - Rs[j])
            right.append(ts[j] - ts[i])
    left = np.array(left).reshape(-1, 3)
    right = np.array(right).reshape(-1, 1)
    print('Left:', left.shape)
    print('Right:', right.shape)
    # 通过最小二乘法求解
    EP_T, _, _, _ = np.linalg.lstsq(left, right, rcond=None)
    print('Got TCP:\n', EP_T)
    return EP_T

def TCF(Ts):
    """
    TCF标定，用于标定旋转矩阵
    :param: Ts :齐次变换矩阵list(len=3)
    :Ts = [点1， 沿+x移动， 沿+z移动]
    """
    assert len(Ts) == 3
    ts = []
    for T in Ts:
        _, t = HomogeneousMtr2Rt(T)
        ts.append(t)
    # 求工具坐标系T的x轴向向量X：
    X = ts[1] - ts[0]
    X = X / np.linalg.norm(X)
    # 求工具坐标系T的z轴向向量Z：
    Z = ts[2] - ts[0]
    Z = Z / np.linalg.norm(Z)
    print('First Z: ', Z)
    # 求工具坐标系T的y轴向向量Y：
    Y = np.cross(Z, X)
    # 重新计算Z：
    first_z = Z
    Z = np.cross(X, Y)
    print('Second Z: ', Z)
    print('First Z and Second Z:', np.dot(first_z, Z))
    # 得到工具坐标T相对于基坐标B的姿态R_BT 
    R_BT = np.vstack((X, Y, Z)).T
    # print('Got RBT:\n', R_BT)
    # 求解工具坐标相对于末端的旋转矩阵
    R_BE, _ = HomogeneousMtr2Rt(Ts[2])
    R_ET = np.linalg.inv(R_BE) @ R_BT

    return R_ET

def get_data():
    # 读取数据
    data_tcp = []
    with open('./input/tcp.txt', 'r') as f:
        for line in f.readlines():
            line = line.strip().split()
            line = [float(x) for x in line]
            data_tcp.append(np.array(line).reshape(4, 4))
    
    data_tcf = []
    with open('./input/tcf.txt', 'r') as f:
        for line in f.readlines():
            line = line.strip().split()
            line = [float(x) for x in line]
            data_tcf.append(np.array(line).reshape(4, 4))
    
    return data_tcp, data_tcf

if __name__ == "__main__":
    data_tcp, data_tcf = get_data()
    t = TCP(data_tcp)
    R = TCF(data_tcf)
    # 组合成齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    print('result:\n', T)
    