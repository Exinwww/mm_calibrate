#ifndef TCP_TCF_H_
#define TCP_TCF_H_

#include<iostream>
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

// 旋转矩阵和平移向量提取
pair<Matrix3d, Vector3d> HomogeneousMtr2Rt(const Matrix4d& H);
// TCP标定
Vector3d TCP(const vector<Matrix4d>& Ts);
// TCF标定
Matrix3d TCF(const vector<Matrix4d>& Ts);

#endif // TCP_TCF_H_