# 需要创建udev规则或sudo运行

import zmq
import time
import json
import threading
import numpy as np
import hid # pip install hidapi


def rotation_matrix(angle, direction, point=None):
    sina = np.sin(angle)
    cosa = np.cos(angle)
    direction = direction / np.linalg.norm(direction)
    R = np.eye(3) * cosa
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array([
        [0.0, -direction[2], direction[1]],
        [direction[2], 0.0, -direction[0]],
        [-direction[1], direction[0], 0.0]
    ])
    M = np.eye(4)
    M[:3, :3] = R
    if point is not None:
        point = np.asarray(point)
        M[:3, 3] = point - np.dot(R, point)
    return M


def to_int16(y1, y2):
    x = (y1) | (y2 << 8)
    if x >= 32768:
        x = -(65536 - x)
    return x


def scale_to_control(x, axis_scale=1.0, min_v=-1.0, max_v=1.0):
    x = x / axis_scale
    x = min(max(x, min_v), max_v)
    return x


def convert(b1, b2):
    return scale_to_control(to_int16(b1, b2))


class SpaceMouse():
    def __init__(
        self,
        vendor_id=9583,
        product_id=50770,
        pos_sensitivity=1.0,
        rot_sensitivity=1.0,
    ):

        print("Opening SpaceMouse device")
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.device = hid.device()
        self.device.open(self.vendor_id, self.product_id)

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        # 6-DOF variables
        self.x, self.y, self.z = 0, 0, 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

        self.single_click_and_hold = False

        self._control = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.rotation = np.array([[-1.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0],
                                  [0.0, 0.0, -1.0]])

        # Launch a new listener thread to listen to SpaceMouse
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()


    def get_controller_state(self):
        dpos = self.control[:3] * self.pos_sensitivity
        roll, pitch, yaw = self.control[3:] * self.rot_sensitivity

        # Convert RPY to an absolute orientation
        drot1 = rotation_matrix(angle=-pitch, direction=[1.0, 0, 0], point=None)[:3, :3]
        drot2 = rotation_matrix(angle=roll, direction=[0, 1.0, 0], point=None)[:3, :3]
        drot3 = rotation_matrix(angle=yaw, direction=[0, 0, 1.0], point=None)[:3, :3]

        self.rotation = self.rotation.dot(drot1.dot(drot2.dot(drot3)))

        return dict(
            dpos=dpos,
            rotation=self.rotation,
            raw_drotation=np.array([roll, pitch, yaw]),
            grasp=self.control_gripper,
        )

    def run(self):

        while True:
            d = self.device.read(13)
            if d is not None:
                if d[0] == 1:  # Readings from 6-DoF sensor
                    self.y = convert(d[1], d[2])
                    self.x = convert(d[3], d[4])
                    self.z = convert(d[5], d[6]) * -1.0

                    self.pitch = convert(d[7], d[8])
                    self.roll = convert(d[9], d[10])
                    self.yaw = -convert(d[11], d[12])

                    self._control = [
                        self.x,
                        self.y,
                        self.z,
                        self.roll,
                        self.pitch,
                        self.yaw,
                    ]

                if d[0] == 3:  # Readings from the side buttons

                    # Press left button
                    if d[1] == 1:
                        self.single_click_and_hold = True

                    # Release left button
                    if d[1] == 0:
                        self.single_click_and_hold = False

    @property
    def control(self):
        return np.array(self._control)

    @property
    def control_gripper(self):
        return -1.0 if self.single_click_and_hold else 1.0


def main_loop(space_mouse):
    # ZeroMQ Configuration
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")  # Modify port if needed

    # Velocity command structure
    velocity_command = {
        "timestamp": 0,
        "linear_velocity": [0.0, 0.0, 0.0],
        "angular_velocity": [0.0, 0.0, 0.0],
        "gripper_velocity": 0.0
    }

    current_linear_velocity = [0.0, 0.0, 0.0]
    current_angular_velocity = [0.0, 0.0, 0.0]

    publish_rate = 50  # Hz
    interval = 1.0 / publish_rate
    ramp_time = 0.4  # seconds for velocity change
    update = 1 / (publish_rate * ramp_time)  # Step size for velocity updates

    while True:
        controller_state = space_mouse.get_controller_state()
        velocity_command["timestamp"] = int(time.time() * 1000)

        # Extract linear and angular velocities from SpaceMouse
        desired_linear_velocity = controller_state["dpos"].tolist()
        desired_angular_velocity = controller_state["raw_drotation"].tolist()

        # Gripper command
        desired_gripper_velocity = controller_state["grasp"]

        # Smoothly update current linear velocities
        for i in range(3):
            delta_linear = desired_linear_velocity[i] - current_linear_velocity[i]
            if abs(delta_linear) > update:
                current_linear_velocity[i] += update * np.sign(delta_linear)
            else:
                current_linear_velocity[i] = desired_linear_velocity[i]
            # Clamp to [-1, 1]
            current_linear_velocity[i] = max(min(current_linear_velocity[i], 1.0), -1.0)

        # Smoothly update current angular velocities
        for i in range(3):
            delta_angular = desired_angular_velocity[i] - current_angular_velocity[i]
            if abs(delta_angular) > update:
                current_angular_velocity[i] += update * np.sign(delta_angular)
            else:
                current_angular_velocity[i] = desired_angular_velocity[i]
            # Clamp to [-1, 1]
            current_angular_velocity[i] = max(min(current_angular_velocity[i], 1.0), -1.0)

        # Update velocity_command
        velocity_command["linear_velocity"] = current_linear_velocity.copy()
        velocity_command["angular_velocity"] = current_angular_velocity.copy()
        velocity_command["cartesian_velocity"] = current_linear_velocity.copy() + current_angular_velocity.copy()

        velocity_command["gripper_velocity"] = desired_gripper_velocity

        # print(velocity_command)

        # Convert to JSON and send via ZeroMQ
        message = json.dumps(velocity_command)
        socket.send_string(message)

        # Wait for the next cycle
        time.sleep(interval)

if __name__ == "__main__":
    space_mouse = SpaceMouse()

    try:
        main_loop(space_mouse)
    except KeyboardInterrupt:
        print("Shutting down.")
