#include <iostream>
#include <unistd.h>
#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <csignal>
#include "json.hpp"
#include "dh_gripper_factory.h"


DH_Gripper* _gripper = nullptr;
int current_position = 500;

std::atomic<float> desired_velocity(0.0);
std::atomic<bool> terminate_program(false);

const float dt = 0.1;
const float max_speed = 5000;
const int force_percent = 20;
const int speed_percent = 100;   

const int max_position = 1000;


// ZeroMQ 订阅线程接收期望的速度
void receiver_thread_func()
{
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5555");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    while (!terminate_program.load())
    {
        zmq::message_t message;
        zmq::recv_result_t received = subscriber.recv(message, zmq::recv_flags::none);

        if (received)
        {
            std::string msg_str(static_cast<char*>(message.data()), message.size());
            nlohmann::json json_msg = nlohmann::json::parse(msg_str);
            if (json_msg.contains("gripper_velocity"))
            {
                float velocity = json_msg["gripper_velocity"];
                desired_velocity.store(velocity);
                // std::cout << "v=" << velocity << std::endl;
            }
        }
    }
}

void control_thread_func()
{
    while (!terminate_program.load())
    {
        float velocity = desired_velocity.load();
        float position_increment = velocity * dt * max_speed;
        current_position += static_cast<int>(position_increment);

        // 将位置限制在有效范围 [0, max_position] 内
        if (current_position < 0)
            current_position = 0;
        if (current_position > max_position)
            current_position = max_position;

        int target_position = current_position;
        // std::cout << "p=" << target_position << std::endl;

        _gripper->SetTargetPosition(target_position);

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
}

int main(int argc, char** argv)
{
    // 可选地处理终止信号（例如 SIGINT）
    std::signal(SIGINT, [](int) { terminate_program.store(true); });

    std::string _gripper_ID = "1";
    std::string _gripper_model = "PGE"; // 似乎与PGI兼容
    std::string _gripper_connect_port = "/dev/ttyUSB0";
    std::string _gripper_Baudrate = "115200";

    DH_Gripper_Factory* _gripper_Factory = new DH_Gripper_Factory();
    _gripper_Factory->Set_Parameter(std::stoi(_gripper_ID), _gripper_connect_port, std::stoi(_gripper_Baudrate));
    _gripper = _gripper_Factory->CreateGripper(_gripper_model);
    if (_gripper == nullptr)
    {
        std::cerr << "无此型号: " << _gripper_model << std::endl;
        return -1;
    }

    if (_gripper->open() < 0)
    {
        std::cerr << "无法打开通信端口: " << _gripper_connect_port << std::endl;
        return -1;
    }

    int initstate = 0;
    _gripper->GetInitState(initstate);
    if (initstate != DH_Gripper::S_INIT_FINISHED)
    {
        _gripper->Initialization();
        std::cout << "正在初始化夹爪..." << std::endl;

        while (initstate != DH_Gripper::S_INIT_FINISHED)
        {
            _gripper->GetInitState(initstate);
            usleep(100000); // 100 毫秒
        }
        std::cout << "夹爪初始化完成" << std::endl;
    }

    _gripper->SetTargetSpeed(speed_percent);
    _gripper->SetTargetForce(force_percent);

    _gripper->GetCurrentPosition(current_position);

    std::thread receiver_thread(receiver_thread_func);
    std::thread control_thread(control_thread_func);

    while (!terminate_program.load())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    terminate_program.store(true);
    if (receiver_thread.joinable())
        receiver_thread.join();
    if (control_thread.joinable())
        control_thread.join();

    _gripper->close();
    delete _gripper_Factory;

    return 0;
}