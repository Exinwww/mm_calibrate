#include <iostream>
#include <time.h>
#include <iterator>
#include <stdio.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <chrono>

#include "fastLoam.h"
#include "yawCalib.h"
#include "rpzCalib.h"

const char usage[] = {
    " ./bin/run_lidar2car <dataset_folder> <output_dir> <start_frame>(optional) <end_frame>(optional)\n"
    "PARAMS:\n"
    "  dataset_folder: path to lidar dataset directory \n"
    "  output_dir: folder to save output files.\n"
    "  start_frame, end_frame: data range. \n"
    "EXAMPLE:\n"
    "  ./bin/run_lidar2car ./data/example ./output/ \n"
    "  ./bin/run_lidar2car ./data/example ./output/ 0 1200\n"};


void SaveExtrinsic(Eigen::Matrix4d T, std::string output_dir)
{
    std::string file_name = output_dir + '/' + "calib_result.txt";
    
    std::ofstream ofs(file_name);
    if (!ofs.is_open())
    {
        std::cerr << "open file " << file_name << " failed. Cannot write calib result." << std::endl;
        exit(1);
    }
    ofs << "Extrinsic = " << std::endl;
    ofs << "[" << T(0, 0) << "," << T(0, 1) << "," << T(0, 2) << "," << T(0, 3) << "],"
        << "[" << T(1, 0) << "," << T(1, 1) << "," << T(1, 2) << "," << T(1, 3) << "],"
        << "[" << T(2, 0) << "," << T(2, 1) << "," << T(2, 2) << "," << T(2, 3) << "],"
        << "[" << T(3, 0) << "," << T(3, 1) << "," << T(3, 2) << "," << T(3, 3) << "]" << std::endl;
    ofs.close();

    std::cout << "Calibration result was saved to file calib_result.txt" << std::endl;
}

int main(int argc, char **argv){
    if (argc != 3 && argc != 5)
    {
        std::cerr << "Usage:" << usage;
        return 1;
    }
    std::string dataset_folder = argv[1];
    std::string output_dir = argv[2];

    std::string outpath = output_dir + "pose.txt";
    std::vector<Eigen::Matrix4d> lidar_pose;
    ReadPose(lidar_pose, outpath);
    YawCalib calibrator2(output_dir);
    calibrator2.LoadData(lidar_pose);

    double yaw = 0;
    if (calibrator2.Calibrate())
    {
        yaw = calibrator2.GetFinalYaw();
        std::cout << "Final result:" << std::endl 
                  << "yaw = " << rad2deg(yaw) << " degree" << std::endl;
    }
    else{
        std::cout << "No valid data for calibrating yaw." << std::endl;
    }

    // read pitch , roll and height
    std::ifstream pr_file;
    pr_file.open(output_dir + "pitch_roll_height.txt");
    if (!pr_file.is_open())
    {
        std::cerr << "open file " << "pitch_roll_height.txt" << " failed. Cannot read pitch and roll." << std::endl;
        exit(1);
    }
    double pitch, roll, height;
    pr_file >> pitch >> roll >> height;
    pr_file.close(); 

    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
    extrinsic(2, 3) = height;
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    extrinsic.block<3, 3>(0, 0) = rotation;
    SaveExtrinsic(extrinsic, output_dir);
}