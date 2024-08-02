# tool2end Calibration
工具坐标系到末端关节的标定

# 原理：使用六点标定法：
* TCP用于标定位置矢量
* TCF用于标定旋转矩阵

参考：[https://blog.csdn.net/qq_41658212/article/details/105686309]

# 编译
```SHELL
mkdir build && cd build
cmake .. && make
```

# 输入
输入数据均为旋转矩阵

* 用于tcp：../data/Tool/tcp.txt
* 用于tcf：../data/Toll/tcf.txt

# 输出
输出工具相对于末端关节的齐次变换矩阵
* ../output/Tool/tool2end.txt

# 运行
```shell
./bin/run_tool2end [input_folder] [output_floder]
```
* 其中input_folder应为存放了tcp.txt和tcf.txt的文件夹，如:

        ./data/Tool/
    若不指定，则默认为上述文件夹

* 其中output_folder将用于存放标定结果tool2end.txt，如:

        ../output/
    若不指定，则默认为上述文件夹
