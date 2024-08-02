#include<iostream>
#include <vector>
#include <Eigen/Dense>
#include "tcp_tcf.h"
using namespace std;
using namespace Eigen;

pair<Matrix3d, Vector3d> HomogeneousMtr2Rt(const Matrix4d& H) {
    /*齐次变换矩阵 提取 旋转矩阵和平移向量*/
    Matrix3d R = H.block<3, 3>(0, 0);
    Vector3d t = H.block<3, 1>(0, 3);
    return make_pair(R, t);
}

Vector3d TCP(const vector<Matrix4d>& Ts){
    /*  
    TCP标定，用于标定位置矢量
    :param：Ts：一系列的齐次变换矩阵
    */
    vector<Matrix3d> Rs;
    vector<Vector3d> ts;
    // 分别提取R和t
    for (const auto& T:Ts){
        auto Rt = HomogeneousMtr2Rt(T);
        Rs.push_back(Rt.first);
        ts.push_back(Rt.second);
    }

    vector<Matrix3d> left;
    vector<Vector3d> right;

    for(size_t i=0; i<Rs.size()-1; i++){
        for(size_t j=i+1; j<Rs.size(); j++){
            left.push_back(Rs[i]-Rs[j]);
            right.push_back(ts[j]-ts[i]);
        }
    }
    MatrixXd leftMat(left.size()*3, 3);
    VectorXd rightVec(right.size()*3);

    for(size_t i=0;i<left.size();++i){
        leftMat.block<3, 3>(i * 3, 0) = left[i];
        rightVec.segment<3>(i * 3) = right[i];
    }
    Vector3d tcp = leftMat.jacobiSvd(ComputeThinU | ComputeThinV).solve(rightVec);
    cout << endl << "resule of TCP: " << tcp.transpose() << endl;
    return tcp;
}

Matrix3d TCF(const vector<Matrix4d>& Ts){
    if(Ts.size() != 3){
        cout << "Get Ts.size = " << Ts.size() << endl;
        cerr << "The number of data is not 3" << endl;
        exit(1);
    }
    vector<Vector3d> ts;
    for (const auto& T:Ts){
        auto Rt = HomogeneousMtr2Rt(T);
        ts.push_back(Rt.second);
    }
    Vector3d X = (ts[1] - ts[0]).normalized();
    Vector3d Z = (ts[2] - ts[0]).normalized();
    cout << "First Z: \t" << Z.transpose() << endl;
    Vector3d Y = Z.cross(X).normalized();

    Vector3d fisrt_Z = Z;
    Z = X.cross(Y).normalized();
    cout << "Second Z:\t" << Z.transpose() << endl;
    cout << "compare Z:\t" << fisrt_Z.dot(Z) << endl;
    // 得到工具坐标系相对于基坐标系B的旋转矩阵R_BT
    Matrix3d R_BT;
    R_BT.col(0) = X;
    R_BT.col(1) = Y;
    R_BT.col(2) = Z;
    // 末端相对于基坐标系的旋转矩阵
    Matrix3d R_BE = HomogeneousMtr2Rt(Ts[2]).first;
    // 计算工具坐标系相对于末端坐标系的旋转矩阵
    Matrix3d R_TE = R_BE.inverse() * R_BT;
    cout << endl << "result of TCF:" << endl << R_TE << endl;
    return R_TE;
}