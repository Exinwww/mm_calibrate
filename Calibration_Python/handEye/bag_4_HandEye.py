#图片中得到标定板相对于RGB相机的位姿

import cv2
import numpy as np
import rospy
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import HandEye
import mm_config
import casadi as ca

def get_rtVecs(imgs:list, cameraMatrix, distCoeffs):
    """得到标定板相对于RGB相机的位姿"""
    # 标定板的尺寸
    checkerboard_size = (11, 8)
    square_size = 35 / 1000

    objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp = objp * square_size

    #
    img_points = []
    obj_points = []
    for img in imgs:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
        # 此处应有判断
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            img_points.append(corners2)
            obj_points.append(objp)
    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], cameraMatrix, distCoeffs)
    return rvecs, tvecs

def bag2Img_and_Joint(bag_path, threshold_diff = 0.0001):
    """
    从bag文件中提取图片和关节信息
    :param bag_path: bag文件路径
    :param threshold_diff: 两张图片之间的时间间隔阈值(单位:ns)
    :return: 对齐结果 aligned_imgs, aligned_joints
    """
    imgs = []
    joints = []

    bag = rosbag.Bag(bag_path)
    # 提取所有的图片和关节信息
    for topic, msg, t in bag.read_messages():
        if topic == '/camera/color/image_raw':
            imgs.append([msg, t.to_sec()])
            # print(t)
        elif topic == '/joint_states':
            # print(t)
            joints.append((msg, t.to_sec() ))
    bag.close()
    # 对齐
    img_ptr, joint_ptr = 0, 0
    aligned_imgs = []
    aligned_joints = []

    while img_ptr < len(imgs) and joint_ptr < len(joints):
        img_msg, img_time = imgs[img_ptr]
        joint_msg, joint_time = joints[joint_ptr]
        diff = img_time - joint_time
        # 小于阈值，视为同一时刻的数据
        if np.abs(diff) < threshold_diff:
            aligned_imgs.append(img_msg)
            aligned_joints.append(joint_msg)
            # 指针后移
            img_ptr += 1
            joint_ptr += 1
        else:
            if diff > 0: # joint滞后
                joint_ptr += 1
            else: # img滞后
                img_ptr += 1

    # print("aligned_imgs: ", len(aligned_imgs))
    # print("aligned_joints: ", len(aligned_joints))

    return aligned_imgs, aligned_joints

def check_imgs(imgs, joints, pattern_size):
    """
    检查图片是否正常，即是否包含完整的标定板
    :param imgs: 图片列表
    :param joints: 关节信息列表
    :param pattern_size: 标定板尺寸
    :return: 检查后的图片列表和关节信息
    """

    new_imgs = []
    new_joints = []

    for i in range(len(imgs)):
        img = imgs[i]
        joint = joints[i]
        # 检查图片是否包含完整的标定板
        try:
            # 转换ROS图像消息到OpenCV格式
            height, width, encoding = img.height, img.width, img.encoding
            img = np.frombuffer(img.data, dtype=np.uint8).reshape(height, width, -1)
            if encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except Exception as e:
            print(e)
            print("Failed to convert image message to cv2 image")
            return None
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, _ = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            new_imgs.append(img)
            new_joints.append(joint.position)
    
    # print("new_imgs: ", len(new_imgs))
    # print("new_joints: ", len(new_joints))
    return new_imgs, new_joints

def getCameraInfo(bag_path):
    """
    从bag文件中提取相机内参, 畸变参数
    """
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages():
        if topic == '/camera/color/camera_info':
            cameraMatrix = np.array(msg.K).reshape(3, 3)
            distCoeffs = np.array(msg.D)
            break
    bag.close()
    return cameraMatrix, distCoeffs

def getHomogeneousMatrix(rvec, tvec):
    """
    从旋转向量和平移向量得到齐次变换矩阵
    """
    R, _ = cv2.Rodrigues(rvec)
    T = np.hstack((R, tvec))
    T = np.vstack((T, np.array([0, 0, 0, 1])))
    return T

def casadi_to_numpy(casadi_matrix, substitution_dict):
    """
    将 CasADi 矩阵中的符号替换为数值，并转换为 NumPy 数组。

    参数:
    casadi_matrix (ca.SX或ca.MX): CasADi 矩阵。
    substitution_dict (dict): 符号到数值的映射字典。

    返回:
    np.ndarray: 转换后的 NumPy 数组。
    """
    # 进行符号替换
    substituted_matrix = casadi_matrix
    for symbol, value in substitution_dict.items():
        substituted_matrix = ca.substitute(substituted_matrix, symbol, value)
    
    # 将替换后的矩阵转换为 DM 矩阵
    dm_matrix = ca.DM(substituted_matrix)
    
    # 将 DM 矩阵转换为 NumPy 数组
    np_array = np.array(dm_matrix)

    return np_array

if __name__ == "__main__":
    checkerboard_size = (11, 8)
    bag_path = './input/2024-08-01-17-05-27.bag'
    imgs, joints = bag2Img_and_Joint(bag_path)
    print(f'Bag for imgs&joints: imgs.size={len(imgs)}, joints.size={len(joints)}')
    imgs, joints = check_imgs(imgs, joints, checkerboard_size)
    print(f'Usable imgs&joints: imgs.size={len(imgs)}, joints.size={len(joints)}')
    # print('joint: ', joints[0])
    # cv2.imshow('img', imgs[0])
    # cv2.waitKey(0)

    # 计算target到cam的齐次变换矩阵
    cameraMatrix, distCoeffs = getCameraInfo(bag_path)
    rvecs, tvecs = get_rtVecs(imgs, cameraMatrix, distCoeffs)
    target2cam = [getHomogeneousMatrix(rvec, tvec)for rvec, tvec in zip(rvecs, tvecs)]
    print(f'Got target2cam.size={len(target2cam)}')

    # 计算末端到base的齐次变换矩阵
    config = mm_config.MMConfig()
    car_state = [0, 0, 0]
    gripper2bases = [config.get_joint_n_trans(car_state, joint, 6) 
                     for joint in joints]
    
    # 定义符号到数值的映射
    at1 = ca.SX.sym('@1')  # 定义符号变量
    substitution_dict = {at1: 0}
    gripper2bases = [casadi_to_numpy(gripper2base, substitution_dict) 
                     for gripper2base in gripper2bases]
    print(f'Got gripper2bases.size={len(gripper2bases)}')

    T_cam2gripper = HandEye.handEye_Calibrate(gripper2bases, target2cam, checkSize=2)
    print('########################################')
    print('HandEye result:')
    print(T_cam2gripper)
    
    # 保存标定结果
    output_path = './output/handEye_result.txt'
    with open(output_path, 'w') as f:
        for i in range(4):
            for j in range(4):
                f.write(str(T_cam2gripper[i][j]) + ' ')
            f.write('\n')

    print('########################################')
    print(f'HandEye result saved into {output_path}')
    print('HandEye calibration done!')
