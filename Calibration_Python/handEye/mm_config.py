import os
import yaml
import casadi as ca
import numpy as np

class MMConfig:
    def __init__(self) -> None:
        with open('mm_param.yaml', 'r', encoding='utf-8') as f:
            self.mm_param = yaml.safe_load(f)
        
        # print(self.mm_param)
        # print(len(self.mm_param['mm']['manipulator_DH_table']))
        
        self.manipulator_dof = self.mm_param['mm']['manipulator_dof']
        self.alpha = []
        self.a = []
        self.theta = []
        self.d = []

        manipulator_DH_table = self.mm_param['mm']['manipulator_DH_table']
        for i in range(self.manipulator_dof):
            self.alpha.append(manipulator_DH_table[i * 4 + 0] / 180.0 * ca.pi)
            self.a.append(manipulator_DH_table[i * 4 + 1])
            self.theta.append(manipulator_DH_table[i * 4 + 2] / 180.0 * ca.pi)
            self.d.append(manipulator_DH_table[i * 4 + 3])

        endlink_gripper_xyz_ypr = self.mm_param['mm']['endlink_gripper_xyz_ypr']
        # print(endlink_gripper_xyz_ypr)
        for i in range(len(endlink_gripper_xyz_ypr)):
            if i > 2:
                endlink_gripper_xyz_ypr[i] = endlink_gripper_xyz_ypr[i] / 180.0 * ca.pi

        self.T_6_g = ca.SX.eye(4)
        R = self.euler_to_rotation_ypr(endlink_gripper_xyz_ypr[3], endlink_gripper_xyz_ypr[4], endlink_gripper_xyz_ypr[5])
        self.T_6_g[:3, :3] = R
        self.T_6_g[:3, 3] = endlink_gripper_xyz_ypr[:3]


        base_mani_fixed_joint_xyz_ypr = self.mm_param['mm']['base_mani_fixed_joint_xyz_ypr']
        # print(endlink_gripper_xyz_ypr)
        for i in range(len(endlink_gripper_xyz_ypr)):
            if i > 2:
                endlink_gripper_xyz_ypr[i] = endlink_gripper_xyz_ypr[i] / 180.0 * ca.pi

        self.T_q_0 = ca.SX.eye(4)
        R = self.euler_to_rotation_ypr(base_mani_fixed_joint_xyz_ypr[3], base_mani_fixed_joint_xyz_ypr[4], base_mani_fixed_joint_xyz_ypr[5])
        self.T_q_0[:3, :3] = R
        self.T_q_0[:3, 3] = base_mani_fixed_joint_xyz_ypr[:3]

        self.gripper_contact_pt_xyz = ca.SX(3, 1)
        for i in range(3):
            self.gripper_contact_pt_xyz[i] = self.mm_param['mm']['endlink_gripper_xyz_ypr'][i]

        self.T_6_contact = ca.SX.eye(4)
        self.T_6_contact[:3, 3] = self.gripper_contact_pt_xyz
        
        # print(self.mm_param['mm']['gripper_contact_pt_xyz'])

    def get_a_joint_tran(self, joint_num, theta):

        st  = ca.sin(theta + self.theta[joint_num])
        ct  = ca.cos(theta + self.theta[joint_num])
        sa  = ca.sin(self.alpha[joint_num])
        ca_ = ca.cos(self.alpha[joint_num])

        T = ca.SX.zeros(4, 4)
        T[0, 0] = ct
        T[0, 1] = -st
        T[0, 2] = 0.0
        T[0, 3] = self.a[joint_num]
        T[1, 0] = st * ca_
        T[1, 1] = ct * ca_
        T[1, 2] = -sa
        T[1, 3] = -sa * self.d[joint_num]
        T[2, 0] = st * sa
        T[2, 1] = ct * sa
        T[2, 2] = ca_
        T[2, 3] = ca_ * self.d[joint_num]
        T[3, 3] = 1.0

        return T

    def get_joint_TMat(self, theta):
        T_joint = []

        for i in range(self.manipulator_dof):
            T_temp = ca.SX(4, 4)
            T_temp = self.get_a_joint_tran(i, theta[i])
            T_joint.append(T_temp)
        
        return T_joint

    def make_identity_mat(self, mat, size):
        rows, cols = mat.shape
        mat_identity = ca.SX.zeros(size * rows, size * cols)

        for i in range(size):
            row_start = i * rows
            col_start = i * cols
            mat_identity[row_start:row_start+rows, col_start:col_start+cols] = mat

        return mat_identity

    def get_yaw_R(self, yaw):
        R = ca.SX.eye(3)
        R[0, 0] =  ca.cos(yaw)
        R[0, 1] = -ca.sin(yaw)
        R[1, 0] =  ca.sin(yaw)
        R[1, 1] =  ca.cos(yaw)
        return R

    def get_joint_n_trans(self, car_state, joint_state, n):
        if n < -1:
            raise ValueError("n must be greater than or equal to -1. -1 for car, 0-6 for manipulator joint")
        
        T_q = ca.SX.zeros(4, 4)
        T_q[0, 0] = ca.cos(car_state[2])
        T_q[0, 1] = -ca.sin(car_state[2])
        T_q[0, 3] = car_state[0]
        T_q[1, 0] = ca.sin(car_state[2])
        T_q[1, 1] = ca.cos(car_state[2])
        T_q[1, 3] = car_state[1]
        T_q[2, 2] = 1.0
        T_q[3, 3] = 1.0

        if n == -1:
            return T_q

        # T_now = T_q @ self.T_q_0
        T_now = ca.SX.eye(4)

        if n == 0:
            return T_now

        for i in range(n):
            temp = self.get_a_joint_tran(i, joint_state[i])
            T_now = T_now @ temp

        return T_now

    def get_gripper_pose(self, car_state, joint_state):
        T_w_6 = self.get_joint_n_trans(car_state, joint_state, 6)
        T_contact = T_w_6 @ self.T_6_contact
        contact_pt = T_contact[:3, 3]
        contact_rot = T_contact[:3, :3]
        contact_quat = self.rotation_matrix_to_quaternion(T_contact[:3, :3])
        return contact_pt, contact_rot, contact_quat

    def rotation_matrix_to_quaternion(self, rot):
        # Ensure the rotation matrix is a CasADi MX type
        rot = ca.SX(rot)
        
        # Extract rotation matrix elements
        r11, r12, r13 = rot[0, 0], rot[0, 1], rot[0, 2]
        r21, r22, r23 = rot[1, 0], rot[1, 1], rot[1, 2]
        r31, r32, r33 = rot[2, 0], rot[2, 1], rot[2, 2]
        
        # Calculate quaternion components
        qw = 0.5 * ca.sqrt(1 + r11 + r22 + r33)
        qx = (r32 - r23) / (4 * qw)
        qy = (r13 - r31) / (4 * qw)
        qz = (r21 - r12) / (4 * qw)
        
        return ca.vertcat(qx, qy, qz, qw)

    def quaternion_to_rotation_matrix(self, q):
        # Ensure the quaternion is a CasADi MX type
        q = ca.SX(q)

        # Extract quaternion elements
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        # Compute the rotation matrix elements
        r11 = 1 - 2*qy**2 - 2*qz**2
        r12 = 2*qx*qy - 2*qz*qw
        r13 = 2*qx*qz + 2*qy*qw

        r21 = 2*qx*qy + 2*qz*qw
        r22 = 1 - 2*qx**2 - 2*qz**2
        r23 = 2*qy*qz - 2*qx*qw

        r31 = 2*qx*qz - 2*qy*qw
        r32 = 2*qy*qz + 2*qx*qw
        r33 = 1 - 2*qx**2 - 2*qy**2

        # Create the rotation matrix
        rot = ca.SX(3, 3)
        rot[0, 0] = r11
        rot[0, 1] = r12
        rot[0, 2] = r13
        rot[1, 0] = r21
        rot[1, 1] = r22
        rot[1, 2] = r23
        rot[2, 0] = r31
        rot[2, 1] = r32
        rot[2, 2] = r33

        return rot

    def xyz_euler_to_rotation(self, xyz_euler): # 内旋xyz欧拉角
        roll = xyz_euler[0]
        pitch = xyz_euler[1]
        yaw = xyz_euler[2]

        cr = ca.cos(roll)
        sr = ca.sin(roll)
        cp = ca.cos(pitch)
        sp = ca.sin(pitch)
        cy = ca.cos(yaw)
        sy = ca.sin(yaw)

        R = ca.SX.zeros(3, 3)
        R[0, 0] =  cp * cy
        R[0, 1] = -cp * sy
        R[0, 2] =  sp
        R[1, 0] =  sr * sp * cy + cr * sy
        R[1, 1] = -sr * sp * sy + cr * cy
        R[1, 2] = -sr * cp
        R[2, 0] = -cr * sp * cy + sr * sy
        R[2, 1] =  cr * sp * sy + sr * cy
        R[2, 2] =  cr * cp
        return R

    def euler_to_rotation_ypr(self, yaw, pitch, roll):
        cy = ca.cos(yaw)
        sy = ca.sin(yaw)
        cp = ca.cos(pitch)
        sp = ca.sin(pitch)
        cr = ca.cos(roll)
        sr = ca.sin(roll)

        R = ca.SX.zeros(3, 3)
        R[0, 0] = cy * cp
        R[0, 1] = cy * sp * sr - sy * cr
        R[0, 2] = cy * sp * cr + sy * sr
        R[1, 0] = sy * cp
        R[1, 1] = sy * sp * sr + cy * cr
        R[1, 2] = sy * sp * cr - cy * sr
        R[2, 0] = -sp
        R[2, 1] = cp * sr
        R[2, 2] = cp * cr
        return R
    

if __name__ == '__main__':
    config = MMConfig()
    car_state = [0, 0, 0]
    joint_state = [0, 0, 0, 0, 0, 0]
    a, b, c = config.get_gripper_pose(car_state, joint_state)
    T_w_6 = config.get_joint_n_trans(car_state, joint_state, 6) # 第 6 个轴末端的位姿
    print(a)
    print(b)