#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <chrono>
#include <string>
#include <zmq.hpp>
#include "json.hpp"
#include "rokae/robot.h"
#include "rokae/utility.h"

using json = nlohmann::json;


int main() {
    const char* zmq_recv_addr = "tcp://localhost:5555";

    // 使用笛卡尔位置控制模式（否则为笛卡尔空间阻抗控制）
    const bool useCartesianPositionMode = true;
    // 使用期望的当前位置（否则为实时查询到的，此时由于传给控制器的位置差总是很小动作会很慢，需要增大最大速度）
    const bool useDesiredPose = true;
    // 解释命令为相对于工具坐标系的移动，否则相对于基座标系
    const bool useTCPMove = true;

    // 假设期望速度在 [-1, 1] 范围内进行标准化
    const double max_linear_velocity = 0.08;  // 最大线速度 (米/秒)
    const double max_angular_velocity = 0.16;  // 最大角速度 (弧度/秒)

    std::cout.setf(std::ios::showpoint);
    std::cout.precision(4);

    std::error_code ec;

    try {
        std::string robot_ip = "192.168.0.160"; // 机器人的 IP 地址
        std::string local_ip = "192.168.0.100"; // 本地机器的 IP 地址

        rokae::xMateErProRobot robot(robot_ip, local_ip);

        // 设置网络容差、操作模式、运动控制模式和电源状态
        robot.setRtNetworkTolerance(10, ec);
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);

        // 实时运动控制器
        auto rtCon = robot.getRtMotionController().lock();

        // 工具在法兰前方 0.2m
        std::array<double, 16> tcp_frame = {1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0.2,
                                            0, 0, 0, 1};
        // std::array<double, 16> tcp_frame = {1, 0, 0, 0,
        //                                     0, 1, 0, 0,
        //                                     0, 0, 1, 0,
        //                                     0, 0, 0, 1};
        // 将被控制坐标系设置为工具坐标系，影响callback应该返回的变换矩阵定义，但不影响robot.posture()返回的姿态，无论ct为何
        rtCon->setEndEffectorFrame(tcp_frame, ec);

        rtCon->setFilterFrequency(25, 25, 52, ec);

        if (useCartesianPositionMode) {
            // 设置关节碰撞检测阈值
            std::array<double, 7> torqueThresholds = {20, 20, 10, 10, 5, 5, 5};
            rtCon->setCollisionBehaviour(torqueThresholds, ec);
        } else {
            rtCon->setFcCoor(tcp_frame, rokae::FrameType::tool, ec);
            rtCon->setCartesianImpedance({1200, 1200, 1200, 100, 100, 100}, ec);
            rtCon->setCartesianImpedanceDesiredTorque({0, 0, 0, 0, 0, 0}, ec);
        }

        // 移动到初始位置
        std::array<double, 7> initial_joint_positions = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};
        rtCon->MoveJ(0.3, robot.jointPos(ec), initial_joint_positions);

        if(!ec){
            std::cout << "初始化成功" << std::endl;
        }

        if (useCartesianPositionMode) {
            rtCon->startMove(rokae::RtControllerMode::cartesianPosition);
        } else {
            rtCon->startMove(rokae::RtControllerMode::cartesianImpedance);
        }

        std::atomic<bool> running(true);

        // zmq 获取的速度命令
        std::mutex command_mutex;
        std::array<double, 3> linear_velocity_cmd = {0.0, 0.0, 0.0};    // [vx, vy, vz]
        std::array<double, 3> angular_velocity_cmd = {0.0, 0.0, 0.0};   // [wx, wy, wz]
        bool command_supressed = false; // 用于暂停控制

        // 跟踪最后一次接收到 zmq 消息的时间
        std::chrono::steady_clock::time_point last_message_time = std::chrono::steady_clock::now();
        const std::chrono::milliseconds timeout_duration(100);

        // 共享变量用于当前姿态
        std::mutex pose_mutex;
        std::array<double, 6> current_posture = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // ZeroMQ 订阅线程接收期望的速度
        auto zmq_receiver = [&]() {
            zmq::context_t context(1);
            zmq::socket_t subscriber(context, ZMQ_SUB);
            subscriber.connect(zmq_recv_addr);
            subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // 订阅所有消息
            std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

            std::array<double, 3> linear_velocity;
            std::array<double, 3> angular_velocity;
            
            while (running) {
                zmq::message_t message;
                zmq::recv_result_t received = subscriber.recv(message, zmq::recv_flags::none); // 阻塞接收消息
                if (received) {
                    std::string msg_str(static_cast<char*>(message.data()), message.size());
                    json msg_json = json::parse(msg_str);

                    if (msg_json.contains("linear_velocity") && msg_json.contains("angular_velocity")) {
                        linear_velocity = msg_json["linear_velocity"].get<std::array<double, 3>>();
                        angular_velocity = msg_json["angular_velocity"].get<std::array<double, 3>>();

                        auto current_time = std::chrono::steady_clock::now();
                        {
                            std::lock_guard<std::mutex> lock(command_mutex);
                            linear_velocity_cmd = linear_velocity;
                            angular_velocity_cmd = angular_velocity;
                            command_supressed = false;
                            last_message_time = current_time;
                        }
                        std::chrono::duration<double, std::milli> elapsed = current_time - last_time;
                        double dt = elapsed.count();
                        last_time = current_time;
                        std::cout << "zmq recv v=[" << linear_velocity[0] << ", " << linear_velocity[1] << ", " << linear_velocity[2] << "] elapsed=" << dt << "ms" << std::endl;
                    }
                }
            }
        };

        std::thread zmq_thread(zmq_receiver);

        // ZeroMQ 发布者线程发送当前末端位置
        auto zmq_sender = [&]() {
            zmq::context_t context(1);
            zmq::socket_t publisher(context, ZMQ_PUB);
            publisher.bind("tcp://*:5556"); // 根据需要调整地址和端口

            while (running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 发送频率为10Hz

                // 复制当前姿态
                std::array<double, 6> posture_copy;
                {
                    std::lock_guard<std::mutex> lock(pose_mutex);
                    posture_copy = current_posture;
                }

                // 创建 JSON 消息
                json msg_json;
                msg_json["position"] = { posture_copy[0], posture_copy[1], posture_copy[2] };
                msg_json["rotation"] = { posture_copy[3], posture_copy[4], posture_copy[5] };

                std::string msg_str = msg_json.dump();

                // 发送消息
                zmq::message_t message(msg_str.size());
                memcpy(message.data(), msg_str.c_str(), msg_str.size());
                publisher.send(message, zmq::send_flags::none);
            }
        };

        // std::thread publisher_thread(zmq_sender);

        // 用于打印日志的变量
        std::array<double, 3> last_pos = {0.0, 0.0, 0.0};
        std::array<double, 3> curr_pos = {0.0, 0.0, 0.0};

        // callback 返回的目标变换矩阵
        std::array<double, 16> current_pose_matrix;
        std::array<double, 6> current_posture_local = robot.posture(rokae::CoordinateType::flangeInBase, ec);
        rokae::Utils::postureToTransArray(current_posture_local, current_pose_matrix);

        // 使用 tcp_frame 计算实际的 current_pose_matrix (tcp 在 base 中)
        std::array<double, 16> tcp_in_base;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
            tcp_in_base[i * 4 + j] = 0;
            for (int k = 0; k < 4; ++k) {
                tcp_in_base[i * 4 + j] += current_pose_matrix[i * 4 + k] * tcp_frame[k * 4 + j];
            }
            }
        }
        current_pose_matrix = tcp_in_base;

        std::function<rokae::CartesianPosition(void)> callback = [&, rtCon]() -> rokae::CartesianPosition {
            auto callback_start = std::chrono::steady_clock::now();

            // 计算时间步长，避免过大值
            // std::chrono::duration<double> elapsed = callback_start - last_time;
            // double dt = elapsed.count();
            // dt = std::min(dt, 0.002);
            // last_time = callback_start;

            double dt = 0.001;

            if(!useDesiredPose){
                auto start1 = std::chrono::steady_clock::now();
                // 获取当前的末端执行器姿态
                std::array<double, 6> current_posture_local = robot.posture(rokae::CoordinateType::flangeInBase, ec);
                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<double, std::milli> dur1 = now - start1;
                rokae::Utils::postureToTransArray(current_posture_local, current_pose_matrix);
                // 更新共享的当前姿态
                // {
                //     std::lock_guard<std::mutex> lock(pose_mutex);
                //     current_posture = current_posture_local;
                // }
                if (dur1.count() > 1){
                    std::cout<< "robot.posture=" << dur1.count() << "ms" << std::endl;
                }
            }

            curr_pos = {current_pose_matrix[3], current_pose_matrix[7], current_pose_matrix[11]};

            // 从姿态矩阵中提取当前的旋转矩阵
            std::array<double, 9> current_rotation_matrix = {
                current_pose_matrix[0], current_pose_matrix[1], current_pose_matrix[2],
                current_pose_matrix[4], current_pose_matrix[5], current_pose_matrix[6],
                current_pose_matrix[8], current_pose_matrix[9], current_pose_matrix[10]
            };

            std::array<double, 3> linear_velocity;
            std::array<double, 3> angular_velocity;
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                auto time_since_last_msg = std::chrono::duration_cast<std::chrono::milliseconds>(callback_start - last_message_time);
                if (time_since_last_msg > timeout_duration && !command_supressed) {
                    // 超时，设置速度为0并输出错误
                    linear_velocity_cmd = {0.0, 0.0, 0.0};
                    angular_velocity_cmd = {0.0, 0.0, 0.0};
                    command_supressed = true;
                    std::cerr << "警告: 未在 " << timeout_duration.count() << " 毫秒内接收到 ZeroMQ 消息。将期望速度置为0。" << std::endl;
                }

                linear_velocity = linear_velocity_cmd;
                angular_velocity = angular_velocity_cmd;
            }

            // TCP（法兰）坐标系的前、左、上分别是z、y、-x
            if(useTCPMove){
                linear_velocity = {-linear_velocity[2], linear_velocity[1], linear_velocity[0]};
                angular_velocity = {-angular_velocity[2], angular_velocity[1], angular_velocity[0]};
            }

            // 计算缩放后的速度
            for (int i = 0; i < 3; ++i) {
                linear_velocity[i] *= max_linear_velocity;
                angular_velocity[i] *= max_angular_velocity;
            }

            // 计算位置变化 (delta_position = linear_velocity * dt)
            std::array<double, 3> delta_position;
            for (int i = 0; i < 3; ++i) {
                delta_position[i] = linear_velocity[i] * dt;
            }

            // 将delta_position从工具坐标系转换到基坐标系
            if (useTCPMove) {
                std::array<double, 3> transformed_delta_position = {0.0, 0.0, 0.0};
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        transformed_delta_position[i] += current_rotation_matrix[i * 3 + j] * delta_position[j];
                    }
                }
                delta_position = transformed_delta_position;
            }

            // 更新姿态矩阵中的位置
            current_pose_matrix[3] += delta_position[0];   // X 位置
            current_pose_matrix[7] += delta_position[1];   // Y 位置
            current_pose_matrix[11] += delta_position[2];  // Z 位置

            // 计算方向变化
            // 将角速度转换为旋转向量
            std::array<double, 3> delta_rotation_vector;
            for (int i = 0; i < 3; ++i) {
                delta_rotation_vector[i] = angular_velocity[i] * dt;
            }

            // 将delta_rotation_vector从工具坐标系转换到基坐标系
            if (useTCPMove) {
                std::array<double, 3> transformed_delta_rotation_vector = {0.0, 0.0, 0.0};
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        transformed_delta_rotation_vector[i] += current_rotation_matrix[i * 3 + j] * delta_rotation_vector[j];
                    }
                }
                delta_rotation_vector = transformed_delta_rotation_vector;
            }

            // 计算旋转角度和轴
            double angle = std::sqrt(delta_rotation_vector[0] * delta_rotation_vector[0] +
                                     delta_rotation_vector[1] * delta_rotation_vector[1] +
                                     delta_rotation_vector[2] * delta_rotation_vector[2]);

            std::array<double, 9> delta_rotation_matrix = {1, 0, 0,
                                                           0, 1, 0,
                                                           0, 0, 1};

            if (angle > 1e-6) {
                // 归一化旋转向量以得到旋转轴
                std::array<double, 3> axis = {delta_rotation_vector[0] / angle,
                                               delta_rotation_vector[1] / angle,
                                               delta_rotation_vector[2] / angle};

                // 使用罗德里格公式计算旋转矩阵
                double c = std::cos(angle);
                double s = std::sin(angle);
                double t = 1 - c;
                double x = axis[0];
                double y = axis[1];
                double z = axis[2];

                delta_rotation_matrix = {
                    t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
                    t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
                    t * x * z - s * y, t * y * z + s * x, t * z * z + c
                };
            }

            // 计算新的旋转矩阵: new_R = delta_R * current_R
            std::array<double, 9> new_rotation_matrix = {0};
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    for (int k = 0; k < 3; ++k) {
                        new_rotation_matrix[i * 3 + j] += delta_rotation_matrix[i * 3 + k] * current_rotation_matrix[k * 3 + j];
                    }
                }
            }

            // 更新姿态矩阵中的旋转矩阵
            current_pose_matrix[0] = new_rotation_matrix[0];
            current_pose_matrix[1] = new_rotation_matrix[1];
            current_pose_matrix[2] = new_rotation_matrix[2];
            current_pose_matrix[4] = new_rotation_matrix[3];
            current_pose_matrix[5] = new_rotation_matrix[4];
            current_pose_matrix[6] = new_rotation_matrix[5];
            current_pose_matrix[8] = new_rotation_matrix[6];
            current_pose_matrix[9] = new_rotation_matrix[7];
            current_pose_matrix[10] = new_rotation_matrix[8];

            // for (int i = 0; i < 4; ++i) {
            //     for (int j = 0; j < 4; ++j) {
            //         std::cout << current_pose_matrix[i * 4 + j] << " ";
            //     }
            //     std::cout << std::endl;
            // }

            rokae::CartesianPosition output{current_pose_matrix};

            // 测量回调执行时间，打印日志
            auto callback_end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> callback_duration = callback_end - callback_start;
            std::cout << "p=[" << curr_pos[0] << ", " << curr_pos[1] << ", " << curr_pos[2] << "] "
            << "dp=[" << delta_position[0] << ", " << delta_position[1] << ", " << delta_position[2] << "] ";
            if(!useDesiredPose){
                std::cout << "rdp=[" << curr_pos[0]-last_pos[0] << ", " << curr_pos[1]-last_pos[1] << ", " << curr_pos[2]-last_pos[2] << "] ";
            }
            std::cout << "dt=" << dt*1000 << "ms exe=" << callback_duration.count() << "ms" << std::endl;
            last_pos = curr_pos;

            return output;
        };

        rtCon->setControlLoop(callback);
        rtCon->startLoop(false);

        std::cout << "开始实时控制，按回车键停止..." << std::endl;
        std::cin.get();

        rtCon->stopLoop();
        std::cout << "控制循环已停止" << std::endl;

        running = false;
        zmq_thread.join();
        // publisher_thread.join();

        robot.setPowerState(false, ec);
    } catch (const std::exception &e) {
        std::cerr << "捕获异常: " << e.what() << "ec=" << ec << std::endl;
    }

    return 0;
}
