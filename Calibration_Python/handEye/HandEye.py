import cv2
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

def hand_eye_calibration(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam):
    """
    手眼标定
    :param: R_gripper2base, t_gripper2base :gripper到base的旋转矩阵和平移向量
    :param: R_target2cam, t_target2cam :target到cam的旋转矩阵和平移向量
    return: R_cam2gripper, t_cam2gripper
    """
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_PARK
    )

    return R_cam2gripper, t_cam2gripper

def handEye_Calibrate(gripper2bases:list, target2cams:list, checkSize=2):
    """
    主函数,手眼标定
    :param: gripper2base 末端关节到base的齐次变换矩阵
    :param: target2cam target到cam的齐次变换矩阵
    """
    assert len(gripper2bases) == len(target2cams), '数据长度不一致'
    print('Start HandEye Calibration...')
    # 记录gripper2base的旋转矩阵和平移向量以及target2cam的旋转矩阵和平移向量
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for i in range(0, len(gripper2bases)):
        R_g2b, t_g2b = HomogeneousMtr2Rt(gripper2bases[i])
        R_gripper2base.append(R_g2b)
        t_gripper2base.append(t_g2b)

        R_t2c, t_t2c = HomogeneousMtr2Rt(target2cams[i])
        R_target2cam.append(R_t2c)
        t_target2cam.append(t_t2c)
    
    # 手眼标定
    R_cam2gripper, t_cam2gripper = hand_eye_calibration(
        R_gripper2base, 
        t_gripper2base, 
        R_target2cam, 
        t_target2cam)
    # 组合为齐次变换矩阵
    T_cam2gripper = np.eye(4)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper.flatten()
    # 检查标定结果
    Check(gripper2bases[:checkSize], target2cams[:checkSize], T_cam2gripper)
    return T_cam2gripper

def Check(gripper2bases, target2cams, T_cam2gripper):
    """
    检查标定结果
    :param: gripper2bases: gripper到base的齐次变换矩阵
    :param: target2cams: target到cam的齐次变换矩阵
    :param: T_cam2gripper: cam到gripper的齐次变换矩阵
    """
    assert len(gripper2bases) == len(target2cams), '数据长度不一致'
    # 使用前两组数据，计算标定板的位姿，可以看到二者基本一致
    print('################ ...Checking... ###############')
    print('Using Calibration Result to compute target2base:')
    for i in range(len(gripper2bases)):
        print(f'{i}:')
        print(gripper2bases[i] @ T_cam2gripper @ target2cams[i])