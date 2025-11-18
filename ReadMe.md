## Apollo10.0 Control Module 

## Instruction
该项目主要参考Apollo10.0控制系统模块对横纵向控制算法进行重构，利用测试数据实现控制指令的单帧解算ComputeControlCommand。内容主要包括横向控制算法[LQR]、纵向控制算法[PID]（位置PID+速度PID）的实现与集成应用，项目中也涉及其他数据处理算法如：[低通二阶滤波器]、[均值滤波器]、[超前/滞后补偿控制器]、[样条插值算法]等的综合应用。项目中间件取消了apollo的cyber通信方式，利用结构体的形式做消息处理。

## build
项目编译，编译结果LatControllerTest、LonControllerTest存放在/build/bin/路径下。
```
mkdir build && cd build && cmake .. -j4 && make -j4
```

[Ctrl+p,Shift+鼠标左键]