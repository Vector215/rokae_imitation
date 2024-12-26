# Rokae Imitation

用于珞石机器人部署模仿学习策略的代码，适用于xMateER7Pro机械臂与大寰PGI夹爪，支持使用笛卡尔速度和笛卡尔/轴空间位置控制，并反馈当前状态

## 编译

在仓库文件夹中准备好xCoreSDK-v0.4.1.b与dh_gripper_ros文件夹

    mkdir build && cd build
    cmake .. && make

## 使用

以下文件之间使用 ZeroMQ 通信

- pub_keyboard.py 使用键盘发送夹爪移动、旋转与开合的控制指令
- pub_spacemouse.py 使用spacemouse发送夹爪移动、旋转与开合的控制指令
- vis_command.py 可视化发送的指令
- all_control 接收指令控制机械臂和夹爪，同时发送状态信息
