#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include "tcp_tcf.h"

using namespace std;
using namespace Eigen;
vector<Matrix4d> readData(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "File not found: " << filename << endl;
        exit(1);
    }
    vector<Matrix4d> Ts;
    while (true) {
        Matrix4d T;
        bool valid = true;
        for (int i = 0; i < 4 && valid; i++) {
            for (int j = 0; j < 4 && valid; j++) {
                if (!(file >> T(i, j))) {
                    valid = false;
                }
            }
        }
        if (!valid) {
            break;  // If reading failed, exit the loop
        }
        Ts.push_back(T);
    }
    cout << "from " << filename << "get: Ts.size = " << Ts.size() << endl;
    file.close();
    return Ts;
}

int main(int argc, char **argv){
    // 输入文件夹与输出文件夹的路径
    string input_folder = "";
    string output_folder = "";

    if(argc == 3){
        cout << "get input and output path from cmd" << endl;
        input_folder = argv[1];
        output_folder = argv[2];
    }else{
        input_folder = "../data/Tool/";
        output_folder = "../output/Tool/";
    }
    cout << "input_folder: " << input_folder << endl;
    cout << "output_folder: " << output_folder << endl;

    string tcp_path = input_folder + "tcp.txt";
    vector<Matrix4d> tcp_data = readData(tcp_path);
    Vector3d tcp = TCP(tcp_data);

    string tcf_path = input_folder + "tcf.txt";
    vector<Matrix4d> tcf_data = readData(tcf_path);
    Matrix3d tcf = TCF(tcf_data);

    // 组合得到最终的标定矩阵
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = tcf;
    T.block<3, 1>(0, 3) = tcp;
    cout << "T: " << endl << T << endl;
    // save
    ofstream file(output_folder + "tool2end.txt");
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            file << T(i, j) << " ";
        }
        file << endl;
    }
    file.close();
}