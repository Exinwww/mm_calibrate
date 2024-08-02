#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include "handEye.h"

using namespace std;
using namespace Eigen;

std::vector<cv::Mat> readData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "File not found: " << filename << std::endl;
        exit(1);
    }

    std::vector<cv::Mat> Ts;
    while (true) {
        cv::Mat T(4, 4, CV_64F);
        bool valid = true;

        for (int i = 0; i < 4 && valid; i++) {
            for (int j = 0; j < 4 && valid; j++) {
                if (!(file >> T.at<double>(i, j))) {
                    valid = false;
                }
            }
        }

        if (!valid) {
            break;  // If reading failed, exit the loop
        }

        Ts.push_back(T);
    }

    std::cout << "from " << filename << " get: Ts.size = " << Ts.size() << std::endl;
    file.close();
    return Ts;
}

int main(int argc, char **argv) {
    // 输入文件夹与输出文件夹的路径
    string input_folder = "";
    string output_folder = "";

    if (argc == 3) {
        cout << "get input and output path from cmd" << endl;
        input_folder = argv[1];
        output_folder = argv[2];
    } else {
        input_folder = "../data/HandEye/";
        output_folder = "../output/HandEye/";
    }
    cout << "input_folder: " << input_folder << endl;
    cout << "output_folder: " << output_folder << endl;

    string gripper2bases_path = input_folder + "gripper2bases.txt";
    vector<cv::Mat> gripper2bases = readData(gripper2bases_path);

    string target2cams_path = input_folder + "target2cams.txt";
    vector<cv::Mat> target2cams = readData(target2cams_path);

    vector<cv::Mat> R_gripper2base;
    vector<cv::Mat> t_gripper2base;
    for (size_t i = 0; i < gripper2bases.size(); i++) {
        auto Rt = HomogeneousMtr2Rt(gripper2bases[i]);
        R_gripper2base.push_back(Rt.first);
        t_gripper2base.push_back(Rt.second);
    }

    vector<cv::Mat> R_target2cam;
    vector<cv::Mat> t_target2cam;
    for (size_t i = 0; i < target2cams.size(); i++) {
        auto Rt = HomogeneousMtr2Rt(target2cams[i]);
        R_target2cam.push_back(Rt.first);
        t_target2cam.push_back(Rt.second);
    }

    cv::Mat H_cam2gripper = hand_eye_calibration(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam);

    Check(gripper2bases, target2cams, H_cam2gripper, 2);

    cout << endl << endl << "H_cam2gripper: " << endl << H_cam2gripper << endl;

    // 保存
    ofstream file(output_folder + "HandEye.txt");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file << H_cam2gripper.at<double>(i, j) << " ";
        }
        file << endl;
    }
    file.close();

    return 0;
}