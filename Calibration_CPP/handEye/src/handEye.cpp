#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include "handEye.h"
#include <stdexcept>

using namespace cv;
using namespace std;

std::pair<cv::Mat, cv::Mat> HomogeneousMtr2Rt(const cv::Mat& H) {
    // 检查输入矩阵 H 的尺寸是否为 4x4
    if (H.rows != 4 || H.cols != 4) {
        throw std::invalid_argument("Input matrix must be 4x4.");
    }

    // 提取旋转矩阵 R (3x3) 和 平移向量 t (3x1)
    cv::Mat R = H(cv::Range(0, 3), cv::Range(0, 3)).clone();
    cv::Mat t = H(cv::Range(0, 3), cv::Range(3, 4)).clone();

    return std::make_pair(R, t);
}

cv::Mat hand_eye_calibration(
    const std::vector<cv::Mat>& R_gripper2base, const std::vector<cv::Mat>& t_gripper2base,
    const std::vector<cv::Mat>& R_target2cam, const std::vector<cv::Mat>& t_target2cam) 
{
    // 检查输入向量的大小是否一致
    if (R_gripper2base.size() != t_gripper2base.size() || 
        R_target2cam.size() != t_target2cam.size() || 
        R_gripper2base.size() != R_target2cam.size()) 
    {
        throw std::invalid_argument("Input vectors must have the same size.");
    }

    // 用于存储结果的旋转矩阵和平移向量
    cv::Mat R_cam2gripper = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat t_cam2gripper = cv::Mat::zeros(3, 1, CV_64F);

    // 调用 OpenCV 的 hand-eye 标定函数
    cv::calibrateHandEye(
        R_gripper2base, t_gripper2base, 
        R_target2cam, t_target2cam, 
        R_cam2gripper, t_cam2gripper, 
        cv::CALIB_HAND_EYE_TSAI
    );

    // 创建 4x4 的齐次变换矩阵并填充
    cv::Mat H_cam2gripper = cv::Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(H_cam2gripper(cv::Range(0, 3), cv::Range(0, 3)));
    t_cam2gripper.copyTo(H_cam2gripper(cv::Range(0, 3), cv::Range(3, 4)));

    return H_cam2gripper;
}

void Check(const std::vector<cv::Mat>& gripper2bases, const std::vector<cv::Mat>& target2cams, const cv::Mat& H_cam2gripper, size_t checkSize) {
    // 确保输入矩阵大小一致
    if (gripper2bases.size() != target2cams.size()) {
        throw std::invalid_argument("Input vectors must have the same size.");
    }
    cout << endl << "Checking the Calibration Result..." << endl;
    for (size_t i = 0; i < min(checkSize,gripper2bases.size()); ++i) {
        // 计算结果矩阵 H
        cv::Mat H = gripper2bases[i] * H_cam2gripper * target2cams[i];

        // 输出结果
        std::cout << endl << "target of [" << i << "]" << std::endl;
        std::cout << H << std::endl;
    }
}