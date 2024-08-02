# 移动操作机器人的标定

包括：

* 手眼标定（眼在手上）：得到Cam相对于末端关节的齐次变换矩阵
    * [https://blog.csdn.net/weixin_45844515/article/details/125571550]

    * [https://blog.csdn.net/hellohake/article/details/104808149]

* 工具系标定（六点标定法）：得到执行器相对于末端关节的齐次变换矩阵
    * [https://blog.csdn.net/qq_41658212/article/details/105686309]

* Lidar2Car标定：得到Lidar相对于小车中心的齐次变换矩阵
    * [https://blog.csdn.net/Walking_roll/article/details/133611662]

### Calibration_CPP
标定的C++实现

* data:输入数据

    * HandEye：手眼的数据

        * gripper2bases：末端关节到底座的齐次变换矩阵
        * target2cams：标定板到相机的齐次变换矩阵
    
    * Tool：工具标定的数据

        * tcf：用于作TCF标定，得到旋转矩阵的数据，应有三个点的末端关节到底座的齐次变换矩阵
        * tcp：用于作TCP标定，得到位置矢量的数据，应有四个点的末端关节到底座的齐次变换矩阵

    * Lidar2Car：Lidar标定的数据

        小车沿直线（推荐）行驶时，采集到的点云数据

* handEye： 手眼标定，使用OpenCV库提供的手眼标定函数

* lidar2car： Lidar标定，仅适用点云数据，即可得到Lidar相对于小车的旋转矩阵和与地面间的高度

* tool2end： 工具系标定，得到工具相对于末端关节的齐次变换矩阵


### Calibration_Python
标定的Python实现

* handEye: input文件夹中给定bag文件，应包含：

    * 相机图像：如/camera/color/image_raw

    * 关节角度：如/joint_states

    * 相机内参：如/camera/color/camera_info

    从bag中提取对齐的图像与关节角信息，分别计算得到gripper2bases和target2cams
    
    再使用OpenCV中的手眼标定函数求解手眼间的齐次变换矩阵T_cam2gripper。

    基于得到的T_cam2gripper，使用Check函数，对每组gripper2base和target2cam，计算标定板相对与base的位姿，基本一致。

* tool_Calibrate: 

    * simulation.py:使用roboticstoolbox-python作仿真，可验证六点标定的正确性

    * Tool_Calibration.py:六点工具标定的实现

        input中给定标定数据：tcf.txt和tcp.txt

        python Tool_calibration.py即可得到标定结果，可查看输出中的：

        “ First Z and Second Z: 0.9999999999999657 “

        当该值接近1时，说明旋转矩阵的标定正确。


