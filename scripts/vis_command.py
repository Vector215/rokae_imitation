import zmq
import json
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


ZMQ_ADDRESS_VEL = "tcp://localhost:5555"  # ZeroMQ 订阅地址 (线速度与角速度)
ZMQ_ADDRESS_ROT = "tcp://localhost:5556"  # ZeroMQ 订阅地址 (固定坐标系旋转)
MAX_LINEAR_VELOCITY = 1.0  # 最大线速度 (m/s)
MAX_ANGULAR_VELOCITY = 2.0  # 最大角速度 (rad/s)
VELOCITY_SCALE = 1.0  # 速度向量的缩放因子
ARROW_LENGTH = 0.5    # 箭头长度
MAX_ROTATION_ANGLE = np.pi / 4  # 最大旋转角度45度

class Pose:
    def __init__(self):
        self.position = np.zeros(3)  # [x, y, z]
        self.orientation = np.eye(3)  # 3x3 旋转矩阵

    def set_velocity(self, linear_velocity, angular_velocity):
        # 设置当前速度对应的姿态
        self.position = linear_velocity * VELOCITY_SCALE

        # 使用罗德里格斯公式计算旋转矩阵，并限制最大旋转角度
        angle = np.linalg.norm(angular_velocity)
        if angle > 1e-6:
            if angle > MAX_ROTATION_ANGLE:
                angular_velocity = angular_velocity / angle * MAX_ROTATION_ANGLE
                angle = MAX_ROTATION_ANGLE
            axis = angular_velocity / np.linalg.norm(angular_velocity)
            self.orientation = self.rotation_matrix(axis, angle)
        else:
            self.orientation = np.eye(3)

    @staticmethod
    def rotation_matrix(axis, angle):
        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c
        return np.array([
            [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
            [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
            [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ]
        ])

# 全局共享变量
desired_linear_velocity = np.zeros(3)    # [vx, vy, vz]
desired_angular_velocity = np.zeros(3)   # [wx, wy, wz]
velocity_lock = threading.Lock()
current_pose = Pose()

fixed_rotation = np.eye(3)  # 固定坐标系的旋转矩阵
rotation_lock = threading.Lock()

def euler_to_rotation(roll, pitch, yaw):
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)

    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [0,   0,  1]])

    Ry = np.array([[cy, 0, sy],
                   [0,  1, 0],
                   [-sy,0, cy]])

    Rx = np.array([[1, 0,    0],
                   [0, cx,  -sx],
                   [0, sx,   cx]])

    R = Rz @ Ry @ Rx
    return R

context = zmq.Context()

def zmq_receiver_vel():
    global desired_linear_velocity, desired_angular_velocity
    socket = context.socket(zmq.SUB)
    socket.connect(ZMQ_ADDRESS_VEL)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        try:
            message = socket.recv_string()
            msg_json = json.loads(message)

            if "linear_velocity" in msg_json and "angular_velocity" in msg_json:
                linear = msg_json["linear_velocity"]
                angular = msg_json["angular_velocity"]

                if len(linear) == 3 and len(angular) == 3:
                    with velocity_lock:
                        desired_linear_velocity = np.array(linear) * MAX_LINEAR_VELOCITY
                        desired_angular_velocity = np.array(angular) * MAX_ANGULAR_VELOCITY
        except Exception as e:
            print(f"Error receiving ZeroMQ message (vel): {e}")

def zmq_receiver_rot():
    global fixed_rotation
    socket = context.socket(zmq.SUB)
    socket.connect(ZMQ_ADDRESS_ROT)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        try:
            message = socket.recv_string()
            msg_json = json.loads(message)

            if "rotation" in msg_json:
                rot = msg_json["rotation"]
                if len(rot) == 3:
                    R = euler_to_rotation(rot[0], rot[1], rot[2])
                    with rotation_lock:
                        fixed_rotation = R
        except Exception as e:
            print(f"Error receiving ZeroMQ message (rot): {e}")

receiver_thread_vel = threading.Thread(target=zmq_receiver_vel, daemon=True)
receiver_thread_vel.start()

# 有bug
# receiver_thread_rot = threading.Thread(target=zmq_receiver_rot, daemon=True)
# receiver_thread_rot.start()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")

max_limit = ARROW_LENGTH * 2
ax.set_xlim([-max_limit, max_limit])
ax.set_ylim([-max_limit, max_limit])
ax.set_zlim([-max_limit, max_limit])

fixed_quivers = {
    'x': ax.quiver(0, 0, 0, ARROW_LENGTH, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.1),
    'y': ax.quiver(0, 0, 0, 0, ARROW_LENGTH, 0, color='g', linewidth=2, arrow_length_ratio=0.1),
    'z': ax.quiver(0, 0, 0, 0, 0, ARROW_LENGTH, color='b', linewidth=2, arrow_length_ratio=0.1)
}

target_quivers = {
    'x': ax.quiver(0, 0, 0, ARROW_LENGTH, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.1),
    'y': ax.quiver(0, 0, 0, 0, ARROW_LENGTH, 0, color='g', linewidth=2, arrow_length_ratio=0.1),
    'z': ax.quiver(0, 0, 0, 0, 0, ARROW_LENGTH, color='b', linewidth=2, arrow_length_ratio=0.1)
}

connection_line, = ax.plot([0, 0], [0, 0], [0, 0], 'k-', linewidth=1)

def animate(frame):
    global current_pose, target_quivers, connection_line, fixed_quivers

    with velocity_lock:
        linear_vel = desired_linear_velocity.copy()
        angular_vel = desired_angular_velocity.copy()

    current_pose.set_velocity(linear_vel, angular_vel)

    R = current_pose.orientation
    t = current_pose.position

    # 获取固定坐标系旋转
    with rotation_lock:
        FR = fixed_rotation.copy()

    # 最终目标坐标系 = 固定坐标系旋转 * 当前动态姿态
    R_target = FR @ R

    arrow_length = ARROW_LENGTH
    # 固定坐标系箭头方向
    fx_dir = FR[:,0] * arrow_length
    fy_dir = FR[:,1] * arrow_length
    fz_dir = FR[:,2] * arrow_length

    fixed_quivers['x'].remove()
    fixed_quivers['y'].remove()
    fixed_quivers['z'].remove()

    fixed_quivers['x'] = ax.quiver(0, 0, 0,
                                   fx_dir[0], fx_dir[1], fx_dir[2],
                                   color='r', linewidth=2, arrow_length_ratio=0.1)
    fixed_quivers['y'] = ax.quiver(0, 0, 0,
                                   fy_dir[0], fy_dir[1], fy_dir[2],
                                   color='g', linewidth=2, arrow_length_ratio=0.1)
    fixed_quivers['z'] = ax.quiver(0, 0, 0,
                                   fz_dir[0], fz_dir[1], fz_dir[2],
                                   color='b', linewidth=2, arrow_length_ratio=0.1)

    # 目标坐标系箭头方向（相对固定坐标系旋转之后）
    x_dir = R_target[:,0] * arrow_length
    y_dir = R_target[:,1] * arrow_length
    z_dir = R_target[:,2] * arrow_length

    # 移除旧的目标箭头
    target_quivers['x'].remove()
    target_quivers['y'].remove()
    target_quivers['z'].remove()

    target_quivers['x'] = ax.quiver(t[0], t[1], t[2],
                                    x_dir[0], x_dir[1], x_dir[2],
                                    color='r', linewidth=2, arrow_length_ratio=0.1)
    target_quivers['y'] = ax.quiver(t[0], t[1], t[2],
                                    y_dir[0], y_dir[1], y_dir[2],
                                    color='g', linewidth=2, arrow_length_ratio=0.1)
    target_quivers['z'] = ax.quiver(t[0], t[1], t[2],
                                    z_dir[0], z_dir[1], z_dir[2],
                                    color='b', linewidth=2, arrow_length_ratio=0.1)

    # 更新连接线
    connection_line.set_data([0, t[0]], [0, t[1]])
    connection_line.set_3d_properties([0, t[2]])

    return list(target_quivers.values()) + [connection_line] + list(fixed_quivers.values())

ani = FuncAnimation(fig, animate, interval=50, blit=False)
plt.show()