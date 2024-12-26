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
#include "dh_gripper_factory.h"

using json = nlohmann::json;

// #define DEBUG


template <std::size_t N>
void multiplyMatrices(const std::array<double, N * N>& A, const std::array<double, N * N>& B, std::array<double, N * N>& result) {
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            result[i * N + j] = 0;
            for (std::size_t k = 0; k < N; ++k) {
                result[i * N + j] += A[i * N + k] * B[k * N + j];
            }
        }
    }
}

void extractXYZRPY(const std::array<double, 16>& transform, std::array<double, 6>& xyzrpy) {
    xyzrpy[0] = transform[3];  // X
    xyzrpy[1] = transform[7];  // Y
    xyzrpy[2] = transform[11]; // Z

    double r00 = transform[0];
    double r01 = transform[1];
    double r10 = transform[4];
    double r11 = transform[5];
    double r20 = transform[8];
    double r21 = transform[9];
    double r22 = transform[10];

    double pitch = std::asin(-r20);
    double cos_pitch = std::cos(pitch);

    const double EPSILON = 1e-6;

    double roll, yaw;

    if (std::abs(cos_pitch) > EPSILON) {
        roll = std::atan2(r21, r22);
        yaw = std::atan2(r10, r00);
    } else {
        // Gimbal lock
        roll = 0.0;
        if (pitch < 0) {
            yaw = std::atan2(-r01, r11);
        } else {
            yaw = std::atan2(r01, r11);
        }
    }

    xyzrpy[3] = roll;
    xyzrpy[4] = pitch;
    xyzrpy[5] = yaw;
}


enum class CmdType {
    xyzrpy_vel, // 接收笛卡尔速度
    joint_pose, // 接收关节角度（期望接收频率接近1000Hz，否则会运动不平滑）
    pose_mat,   // 接收tcp变换矩阵（期望接收频率接近1000Hz，否则会运动不平滑）
};


int main() {
    std::cout.setf(std::ios::showpoint);
    std::cout.precision(4);
    const bool logging = false;
    const char* zmq_recv_addr = "tcp://localhost:5555";
    const char* zmq_pub_addr = "tcp://localhost:5556";

    // 使用位置控制模式，否则为阻抗控制（阻抗控制需要不装工具或有工具标定数据，后者暂时没有）
    const bool usePositionControl = true;
    // （仅在xyzrpy_vel时有效）使用期望的当前位置，否则为实时查询到的，此时由于传给机械臂的位置差总是很小动作会很慢，需要增大最大速度
    const bool useDesiredPose = true;
    // （仅在xyzrpy_vel时有效）解释命令为相对于工具坐标系的移动，否则相对于基座标系
    const bool useTCPMove = true;

    const CmdType cmdType = CmdType::xyzrpy_vel;

    // xyzrpy_vel时的最大速度，假设期望速度在 [-1, 1] 范围内进行归一化
    const double max_linear_velocity = 0.06;   // 最大线速度 (米/秒)
    const double max_angular_velocity = 0.10;  // 最大角速度 (弧度/秒)

    // 夹爪控制参数
    const bool use_gripper = false;
    const float gripper_max_speed = 5000;
    const int gripper_speed_percent = 100;
    const int gripper_force_percent = 20;
    const int gripper_position_max = 1000; // 固定值

    const std::chrono::milliseconds gripper_control_duration(100); // 夹爪控制的时间间隔
    const std::chrono::milliseconds zmq_recv_timeout(100);         // zmq 接收命令的超时时间，超时后忽略速度命令
    const std::chrono::milliseconds zmq_pub_duration(50);          // zmq 发送机器人状态的间隔时间
    std::chrono::steady_clock::time_point last_message_time;       // 最后一次接收到 zmq 消息的时间
    last_message_time = std::chrono::steady_clock::now();

    // zmq 获取的命令
    std::mutex command_mutex;
    std::array<double, 6> cartesian_velocity_cmd = {0.0};
    std::array<double, 16> pose_matrix_cmd;
    std::vector<double> joint_position_cmd;
    std::atomic<float> gripper_velocity_cmd = 0.0;
    std::atomic<bool> command_supressed = false; // 用于在 zmq 超时时忽略速度命令，位置命令不更新只会停下是安全的
    std::atomic<bool> running = true;

    // zmq 发布的当前姿态
    std::mutex pose_mutex;
    std::array<double, 6> current_posture;
    std::array<double, 7> current_joint;
    std::atomic<int> gripper_position;

    // 夹爪
    std::string gripper_port = "/dev/ttyUSB0";
    DH_Gripper_Factory gripper_factory;
    gripper_factory.Set_Parameter(1, gripper_port, 115200);
    DH_Gripper* gripper = gripper_factory.CreateGripper(std::string("PGE"));

    auto gripper_controller = [&]() {
        // 初始化夹爪
        if (gripper->open() < 0){
            std::cerr << "无法打开通信端口: " << gripper_port << std::endl;
            return -1;
        }
        int initstate = 0;
        gripper->GetInitState(initstate);
        if (initstate != DH_Gripper::S_INIT_FINISHED) {
            gripper->Initialization();

            while (initstate != DH_Gripper::S_INIT_FINISHED) {
                gripper->GetInitState(initstate);
                std::this_thread::sleep_for(gripper_control_duration);
            }
        }

        // 设置夹爪参数 
        gripper->SetTargetSpeed(gripper_speed_percent);
        gripper->SetTargetForce(gripper_force_percent);
        int gripper_pos_temp, target_position;
        float dt = gripper_control_duration.count() / 1000.0;

        int target_gripper_position = gripper_position_max;

        while (running) {
            gripper->GetCurrentPosition(gripper_pos_temp);
            gripper_position = gripper_pos_temp;

            float delta_position = gripper_velocity_cmd * dt * gripper_max_speed;
            target_gripper_position += static_cast<int>(delta_position);

            // 将位置限制在有效范围 [0, max_position] 内
            if (target_gripper_position < 0) {
                target_gripper_position = 0;
            } else if (target_gripper_position > gripper_position_max) {
                target_gripper_position = gripper_position_max;
            }

            gripper->SetTargetPosition(target_gripper_position);

            std::this_thread::sleep_for(gripper_control_duration);
        }
    };

    if (use_gripper) {
        std::thread control_thread(gripper_controller);
    }

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
        std::shared_ptr<rokae::RtMotionControlCobot<7>> rtCon = robot.getRtMotionController().lock();

        // 工具中心点在法兰前方0.25m，坐标系相同
        std::array<double, 16> tcp_frame = {1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0.25,
                                            0, 0, 0, 1};
        // std::array<double, 16> tcp_frame = {1, 0, 0, 0,
        //                                     0, 1, 0, 0,
        //                                     0, 0, 1, 0,
        //                                     0, 0, 0, 1};
        // 将被控制坐标系设置为工具坐标系，影响callback应该返回的变换矩阵定义，但不影响robot.posture()返回的姿态，无论ct为何
        rtCon->setEndEffectorFrame(tcp_frame, ec);

        rtCon->setFilterFrequency(25, 25, 52, ec);

        if (usePositionControl) {
            // 设置碰撞检测阈值
            rtCon->setCollisionBehaviour({16, 16, 8, 8, 4, 4, 4}, ec);
        } else {
            // 设置阻抗系数
            rtCon->setFcCoor(tcp_frame, rokae::FrameType::tool, ec);
            if (cmdType == CmdType::xyzrpy_vel || cmdType == CmdType::pose_mat) {
                rtCon->setCartesianImpedance({1200, 1200, 1200, 100, 100, 100}, ec);
                // rtCon->setCartesianImpedanceDesiredTorque({0, 0, 0, 0, 0, 0}, ec);
            } else if (cmdType == CmdType::joint_pose) {
                rtCon->setJointImpedance({1200, 1200, 1200, 100, 100, 100, 100}, ec);
            }
        }

        // 移动到初始位置
        std::array<double, 7> initial_joint_positions = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};
        rtCon->MoveJ(0.3, robot.jointPos(ec), initial_joint_positions);

        if(ec){
            std::cerr << "初始化失败 ec=" << ec << std::endl;
            return 0;
        }

        // 设置对应的控制模式
        if (usePositionControl) {
            if (cmdType == CmdType::xyzrpy_vel || cmdType == CmdType::pose_mat) {
                rtCon->startMove(rokae::RtControllerMode::cartesianPosition);
            } else if (cmdType == CmdType::joint_pose) {
                rtCon->startMove(rokae::RtControllerMode::jointPosition);
            }
        } else {
            if (cmdType == CmdType::xyzrpy_vel || cmdType == CmdType::pose_mat) {
                rtCon->startMove(rokae::RtControllerMode::cartesianImpedance);
            } else if (cmdType == CmdType::joint_pose) {
                rtCon->startMove(rokae::RtControllerMode::jointImpedance);
            }
        }

        // zmq 收期望的速度
        auto zmq_receiver = [&]() {
            zmq::context_t context(1);
            zmq::socket_t subscriber(context, ZMQ_SUB);
            subscriber.connect(zmq_recv_addr);
            subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0); // 订阅所有消息
            std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

            std::array<double, 6> cartesian_velocity;
            std::array<double, 16> pose_matrix;
            std::array<double, 7> joint_position;
            
            while (running) {
                zmq::message_t message;
                zmq::recv_result_t received = subscriber.recv(message, zmq::recv_flags::dontwait); // 不阻塞
                if (received) {
                    auto current_time = std::chrono::steady_clock::now();
                    std::chrono::duration<double, std::milli> elapsed = current_time - last_time;
                    double dt = elapsed.count();
                    last_time = current_time;

                    std::string msg_str(static_cast<char*>(message.data()), message.size());
                    json msg_json = json::parse(msg_str);

                    if (msg_json.contains("cartesian_velocity")) {
                        cartesian_velocity = msg_json["cartesian_velocity"].get<std::array<double, 6>>();
                        {
                            std::lock_guard<std::mutex> lock(command_mutex);
                            cartesian_velocity_cmd = cartesian_velocity;
                            command_supressed = false;
                            last_message_time = current_time;
                        }

                        #ifdef DEBUG
                        std::cout << "zmq recv v=[" << cartesian_velocity[0] << ", " << cartesian_velocity[1] 
                                    << ", " << cartesian_velocity[2] << "]"<< cartesian_velocity[3] << ", " << cartesian_velocity[4] 
                                    << ", " << cartesian_velocity[5] <<" elapsed=" << dt << "ms" << std::endl;
                        #endif
                    } else if (msg_json.contains("pose_matrix")){
                        pose_matrix = msg_json["pose_matrix"].get<std::array<double, 16>>();
                        {
                            std::lock_guard<std::mutex> lock(command_mutex);
                            pose_matrix_cmd = pose_matrix;
                            last_message_time = current_time;
                        }
                    } else if (msg_json.contains("joint_position")){
                        joint_position = msg_json["joint_position"].get<std::array<double, 7>>();
                        {
                            std::lock_guard<std::mutex> lock(command_mutex);
                            joint_position_cmd.clear();
                            std::copy(joint_position.begin(), joint_position.end(), std::back_inserter(joint_position_cmd));
                            last_message_time = current_time;
                        }
                    } else {
                        std::cerr << "未知的zmq控制命令" << msg_json << std::endl;
                    }

                    if (msg_json.contains("gripper_velocity"))
                    {
                        gripper_velocity_cmd = msg_json["gripper_velocity"];
                    }
                }
            }
        };

        // zmq 发送当前状态
        auto zmq_sender = [&]() {
            zmq::context_t context(1);
            zmq::socket_t publisher(context, ZMQ_PUB);
            publisher.bind(zmq_pub_addr);

            while (running) {
                // 复制当前姿态
                std::array<double, 6> posture_copy;
                std::array<double, 7> joint_copy;
                double gripper_copy;
                {
                    std::lock_guard<std::mutex> lock(pose_mutex);
                    posture_copy = current_posture;
                    joint_copy = current_joint;
                    gripper_copy = gripper_position / gripper_position_max;
                }

                // 创建 JSON 消息
                json msg_json;
                msg_json["ActualTCPPose"] = {posture_copy[0], posture_copy[1], posture_copy[2],posture_copy[3], posture_copy[4], posture_copy[5]};
                msg_json["ActualJointPose"] = {joint_copy[0], joint_copy[1], joint_copy[2], joint_copy[3], joint_copy[4], joint_copy[5], joint_copy[6]};
                msg_json["ActualGripperPose"] = gripper_copy;

                std::string msg_str = msg_json.dump();
                // std::cout<<msg_str<<"\n";
                // 发送消息
                zmq::message_t message(msg_str.size());
                memcpy(message.data(), msg_str.c_str(), msg_str.size());
                publisher.send(message, zmq::send_flags::none);

                std::this_thread::sleep_for(zmq_pub_duration);
            }
        };

        // 用于打印日志的变量
        std::array<double, 3> last_pos = {0.0};
        std::array<double, 3> curr_pos = {0.0};

        // callback_cart 返回的目标变换矩阵，使用当前值初始化
        std::array<double, 16> target_pose_matrix;
        std::array<double, 6> current_posture_flange;
        current_posture_flange = robot.posture(rokae::CoordinateType::flangeInBase, ec);
        rokae::Utils::postureToTransArray(current_posture_flange, target_pose_matrix);
        // 使用 tcp_frame 计算当前 tcp 在 base 中的坐标来初始化 target_pose_matrix 和 current_posture
        std::array<double, 16> tcp_in_base;
        multiplyMatrices<4>(target_pose_matrix, tcp_frame, tcp_in_base);
        target_pose_matrix = tcp_in_base;
        extractXYZRPY(tcp_in_base, current_posture);

        // callback_joint 返回的关节角度，使用当前值初始化
        std::vector<double> target_joint_pose;
        current_joint = robot.jointPos(ec);
        std::copy(current_joint.begin(), current_joint.end(), std::back_inserter(target_joint_pose));

        // 同时初始化位置控制命令为当前位置
        pose_matrix_cmd = target_pose_matrix;
        joint_position_cmd = target_joint_pose;

        // 启动 zmq 发布和订阅线程
        std::thread zmq_sender_thread(zmq_sender);
        std::thread zmq_receiver_thread(zmq_receiver);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 轴空间控制时的回调函数
        std::function<rokae::JointPosition()> callback_joint = [&, rtCon]() {
            auto start1 = std::chrono::steady_clock::now();
            // 获取当前的机器人状态：末端执行器姿态/轴角
            // TODO 耗时可能比较长，切换为setControlLoop(useStateDataInLoop=True)和getStateData
            std::array<double, 6> current_posture_flange = robot.posture(rokae::CoordinateType::flangeInBase, ec);
            std::array<double, 16> current_mat_temp;
            rokae::Utils::postureToTransArray(current_posture_flange, current_mat_temp);
            std::array<double, 16> tcp_in_base;
            multiplyMatrices<4>(current_mat_temp, tcp_frame, tcp_in_base);
            std::array<double, 6> current_posture_temp;
            extractXYZRPY(tcp_in_base, current_posture_temp);

            std::array<double, 7> joint_pose_temp = robot.jointPos(ec);
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> dur1 = now - start1;
            if (dur1.count() > 1){
                std::cout<< "robot.posture=" << dur1.count() << "ms" << std::endl;
            }

            // 更新共享的当前姿态
            {
                std::lock_guard<std::mutex> lock(pose_mutex);
                current_posture = current_posture_temp;
                current_joint = joint_pose_temp;
            }

            // 获取关节位置直接返回
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                target_joint_pose = joint_position_cmd;
            }
            return rokae::JointPosition(target_joint_pose);
        };

        // 笛卡尔空间控制时的回调函数
        std::function<rokae::CartesianPosition()> callback_cart = [&, rtCon]() {
            std::chrono::steady_clock::time_point callback_start = std::chrono::steady_clock::now();

            double dt = 0.001; // 尽管回调间隔可能不是 1ms

            auto start1 = std::chrono::steady_clock::now();
            // 获取当前的机器人状态：末端执行器姿态/轴角
            // TODO 耗时可能比较长，切换为setControlLoop(useStateDataInLoop=True)和getStateData
            std::array<double, 6> current_posture_flange = robot.posture(rokae::CoordinateType::flangeInBase, ec);
            std::array<double, 16> current_mat_temp;
            rokae::Utils::postureToTransArray(current_posture_flange, current_mat_temp);
            std::array<double, 16> tcp_in_base;
            multiplyMatrices<4>(current_mat_temp, tcp_frame, tcp_in_base);
            std::array<double, 6> current_posture_temp;
            extractXYZRPY(tcp_in_base, current_posture_temp);

            std::array<double, 7> joint_pose_temp = robot.jointPos(ec);
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> dur1 = now - start1;
            if (dur1.count() > 1){
                std::cout<< "robot.posture=" << dur1.count() << "ms" << std::endl;
            }

            // 更新共享的当前姿态
            {
                std::lock_guard<std::mutex> lock(pose_mutex);
                current_posture = current_posture_temp;
                current_joint = joint_pose_temp;
            }

            // 接收变换矩阵时直接返回
            if (cmdType == CmdType::pose_mat){
                {
                    std::lock_guard<std::mutex> lock(command_mutex);
                    target_pose_matrix = pose_matrix_cmd;
                }
                return rokae::CartesianPosition(target_pose_matrix);
            }

            // 使用实时查询到的位置作为位置变换起点
            if(!useDesiredPose){
                target_pose_matrix = tcp_in_base;
            }

            curr_pos = {target_pose_matrix[3], target_pose_matrix[7], target_pose_matrix[11]};

            // 从姿态矩阵中提取当前的旋转矩阵
            std::array<double, 9> current_rotation_matrix = {
                target_pose_matrix[0], target_pose_matrix[1], target_pose_matrix[2],
                target_pose_matrix[4], target_pose_matrix[5], target_pose_matrix[6],
                target_pose_matrix[8], target_pose_matrix[9], target_pose_matrix[10]
            };

            std::array<double, 6> velocity;
            std::array<double, 3> linear_velocity;
            std::array<double, 3> angular_velocity;
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                auto time_since_last_msg = std::chrono::duration_cast<std::chrono::milliseconds>(callback_start - last_message_time);
                if (time_since_last_msg > zmq_recv_timeout && !command_supressed) {
                    // 超时，设置速度为0并输出错误
                    cartesian_velocity_cmd = std::array<double, 6>{0.0};
                    command_supressed = true;
                    std::cerr << "警告: 未在 " << zmq_recv_timeout.count() << " 毫秒内接收到 zmq 消息。将期望速度置为0。" << std::endl;
                }

                velocity = cartesian_velocity_cmd;
            }

            // TCP（法兰）坐标系的前、左、上分别是z、y、-x
            if(useTCPMove){
                linear_velocity = {-velocity[2], velocity[1], velocity[0]};
                angular_velocity = {-velocity[5], velocity[4], velocity[3]};
            } else {
                linear_velocity = {velocity[0], velocity[1], velocity[2]};
                angular_velocity = {velocity[3], velocity[4], velocity[5]};
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
            target_pose_matrix[3] += delta_position[0];   // X 位置
            target_pose_matrix[7] += delta_position[1];   // Y 位置
            target_pose_matrix[11] += delta_position[2];  // Z 位置

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
                    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
                };
            }

            // 计算新的旋转矩阵: new_R = delta_R * current_R
            std::array<double, 9> new_rotation_matrix = {0.0};
            multiplyMatrices<3>(delta_rotation_matrix, current_rotation_matrix, new_rotation_matrix);

            // 更新姿态矩阵中的旋转矩阵
            target_pose_matrix[0] = new_rotation_matrix[0];
            target_pose_matrix[1] = new_rotation_matrix[1];
            target_pose_matrix[2] = new_rotation_matrix[2];
            target_pose_matrix[4] = new_rotation_matrix[3];
            target_pose_matrix[5] = new_rotation_matrix[4];
            target_pose_matrix[6] = new_rotation_matrix[5];
            target_pose_matrix[8] = new_rotation_matrix[6];
            target_pose_matrix[9] = new_rotation_matrix[7];
            target_pose_matrix[10] = new_rotation_matrix[8];

            // 测量回调执行时间，打印日志
            #ifdef DEBUG
            std::cout << "target_pose_matrix:" << std::endl;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    std::cout << target_pose_matrix[i * 4 + j] << " ";
                }
                std::cout << std::endl;
            }
            std::chrono::steady_clock::time_point callback_end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> callback_duration = callback_end - callback_start;
            std::cout << "p=[" << curr_pos[0] << ", " << curr_pos[1] << ", " << curr_pos[2] << "] ";
            std::cout << "dp=[" << delta_position[0] << ", " << delta_position[1] << ", " << delta_position[2] << "] ";
            if(!useDesiredPose){
                std::cout << "rdp=[" << curr_pos[0]-last_pos[0] << ", " << curr_pos[1]-last_pos[1] << ", " << curr_pos[2]-last_pos[2] << "] ";
            }
            std::cout << "dt=" << dt*1000 << "ms exe=" << callback_duration.count() << "ms" << std::endl;
            last_pos = curr_pos;
            #endif

            return rokae::CartesianPosition(target_pose_matrix);
        };

        if (cmdType == CmdType::joint_pose) {
            rtCon->setControlLoop(callback_joint);
        } else {
            rtCon->setControlLoop(callback_cart);
        };
        rtCon->startLoop(false);

        std::cout << "开始实时控制，按回车键停止..." << std::endl;
        std::cin.get();

        rtCon->stopLoop();
        std::cout << "控制循环已停止" << std::endl;

        running = false;
        zmq_receiver_thread.join();
        zmq_sender_thread.join();

        robot.setPowerState(false, ec); // TODO 无法下电
    } catch (const std::exception &e) {
        std::cerr << "捕获异常: " << e.what() << " ec=" << ec << std::endl;
    }

    return 0;
}
