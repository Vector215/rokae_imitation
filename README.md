# Rokae Imitation

用于珞石机器人部署模仿学习策略的代码，适用于xMateER7Pro机械臂与大寰PGI夹爪

## 编译

在硬盘上准备好xCoreSDK与dh_gripper_ros软件包，将其路径存入CMakeLists.txt中的对应位置

## 使用

- pub_keyboard.py 使用键盘发送夹爪移动、旋转与开合的控制指令
- vis_command.py 可视化发送的指令
- arm_control.py 接受控制指令并控制机械臂运动
- gripper_control.py 接受控制指令并控制夹爪运动
