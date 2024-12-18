import zmq
import time
import json
import threading
from pynput import keyboard

# ZeroMQ 配置
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")  # 根据需要修改端口

# 速度指令，初始为零
velocity_command = {
    "timestamp": 0,
    "linear_velocity": [0.0, 0.0, 0.0],
    "angular_velocity": [0.0, 0.0, 0.0],
    "gripper_velocity": 0.0
}

# 初始化当前速度和目标速度
current_linear_velocity = [0.0, 0.0, 0.0]
current_angular_velocity = [0.0, 0.0, 0.0]
current_gripper_velocity = 0.0

# 键位与速度的映射
key_velocity_map = {
    'w': ('linear', 0, 1.0),   # 前进（1轴正方向）
    's': ('linear', 0, -1.0),  # 后退
    'a': ('linear', 1, 1.0),   # 左移（2轴正方向）
    'd': ('linear', 1, -1.0),  # 右移
    'r': ('linear', 2, 1.0),   # 上升（3轴正方向）
    'f': ('linear', 2, -1.0),  # 下降
    'l': ('angular', 0, 1.0),  # 绕 1 轴正向旋转
    'j': ('angular', 0, -1.0), # 绕 1 轴反向旋转
    'i': ('angular', 1, 1.0),  # 绕 2 轴正向旋转
    'k': ('angular', 1, -1.0), # 绕 2 轴反向旋转
    'u': ('angular', 2, 1.0),  # 绕 3 轴正向旋转
    'o': ('angular', 2, -1.0), # 绕 3 轴反向旋转
    "'": ('gripper', None, 1.0),  # 开夹爪（增加宽度）
    ";": ('gripper', None, -1.0)  # 关夹爪（减少宽度）
}

# 当前按下的键集合
pressed_keys = set()

def on_press(key):
    try:
        k = key.char
        if k in key_velocity_map:
            pressed_keys.add(k)
    except AttributeError:
        pass

def on_release(key):
    try:
        k = key.char
        if k in key_velocity_map and k in pressed_keys:
            pressed_keys.remove(k)
    except AttributeError:
        pass

def keyboard_listener():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

# 启动键盘监听线程
keyboard_thread = threading.Thread(target=keyboard_listener)
keyboard_thread.daemon = True
keyboard_thread.start()

# 主循环，持续发送速度指令
def main_loop():
    global current_gripper_velocity
    publish_rate = 50  # 发布频率（Hz）
    interval = 1.0 / publish_rate
    ramp_time = 0.4  # 速度变化时间（秒）
    update = 1 / (publish_rate * ramp_time)  # 更新速度的幅度

    while True:
        desired_linear_velocity = [0.0, 0.0, 0.0]
        desired_angular_velocity = [0.0, 0.0, 0.0]
        desired_gripper_velocity = 0.0
        velocity_command["timestamp"] = int(time.time() * 1000)

        # 根据当前按下的键更新目标速度
        for k in pressed_keys:
            motion_type, axis, value = key_velocity_map[k]
            if motion_type == 'linear':
                desired_linear_velocity[axis] += value
            elif motion_type == 'angular':
                desired_angular_velocity[axis] += value
            elif motion_type == 'gripper':
                desired_gripper_velocity += value

        # 平滑更新当前速度（使用固定步长update）
        for i in range(3):
            # 计算当前速度和目标速度之间的差值
            delta_linear = desired_linear_velocity[i] - current_linear_velocity[i]
            delta_angular = desired_angular_velocity[i] - current_angular_velocity[i]

            # 朝着目标方向移动步长，如果差值小于步长直接设置为目标值
            if abs(delta_linear) > update:
                current_linear_velocity[i] += update * (delta_linear / abs(delta_linear))
            else:
                current_linear_velocity[i] = desired_linear_velocity[i]

            if abs(delta_angular) > update:
                current_angular_velocity[i] += update * (delta_angular / abs(delta_angular))
            else:
                current_angular_velocity[i] = desired_angular_velocity[i]

            # 归一化速度值到 [-1, 1]
            current_linear_velocity[i] = max(min(current_linear_velocity[i], 1.0), -1.0)
            current_angular_velocity[i] = max(min(current_angular_velocity[i], 1.0), -1.0)


        # 更新速度指令
        velocity_command["linear_velocity"] = current_linear_velocity.copy()
        velocity_command["angular_velocity"] = current_angular_velocity.copy()

        # 更新夹爪指令
        delta_gripper_width = desired_gripper_velocity - current_gripper_velocity
        if abs(delta_gripper_width) > update:
            current_gripper_velocity += update * (delta_gripper_width / abs(delta_gripper_width))
        else:
            current_gripper_velocity = desired_gripper_velocity
        current_gripper_velocity = max(min(current_gripper_velocity, 1.0), -1.0)
        velocity_command["gripper_velocity"] = current_gripper_velocity

        # 将指令转换为 JSON 格式并发送
        message = json.dumps(velocity_command)
        socket.send_string(message)

        # 等待下一个周期
        time.sleep(interval)

if __name__ == "__main__":
    main_loop()
