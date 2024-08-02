#ifndef HAND_EYE_H_
#define HAND_EYE_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include <stdexcept>

using namespace cv;
using namespace std;

cv::Mat hand_eye_calibration(
    const std::vector<cv::Mat>& R_gripper2base, const std::vector<cv::Mat>& t_gripper2base,
    const std::vector<cv::Mat>& R_target2cam, const std::vector<cv::Mat>& t_target2cam);
std::pair<cv::Mat, cv::Mat> HomogeneousMtr2Rt(const cv::Mat& H);
void Check(const std::vector<cv::Mat>& gripper2bases, const std::vector<cv::Mat>& target2cams, const cv::Mat& H_cam2gripper, size_t checkSize);

#endif // HAND_EYE_H_