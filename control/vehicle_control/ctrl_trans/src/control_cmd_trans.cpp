extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include "Controller.h"
#include <chrono>
#include <thread>
#include <ctime>
#include <sys/time.h>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdint>
#include <iostream>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;


struct Trq_Bre_Cmd {
    uint8_t bre_enable;
    float bre_value;
    uint8_t trq_enable;
    float trq_value;
};

int init_udp_socket(const char* ip, int port) {
    // 创建 UDP 套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }
    // 目标地址和端口
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported" << std::endl;
        close(sockfd);
        return -1;
    }

    return sockfd;
}

int sockfd = init_udp_socket("192.168.1.101", 23456);//目标IP及端口
float steering_angle = 0.0;
struct Trq_Bre_Cmd temp;
bool trq_update = false;
bool steer_update = false;

// 范围限制函数
// float clamp(float value, float min, float max) {
//     return (value < min) ? min : (value > max) ? max : value;
// }

// 线性映射函数 - 改进版本，增加边界保护
float mapToRange(float value, float inMin, float inMax) {
    // 边界保护，防止除零错误
    if (inMax == inMin) return 0.0f;
    
    // 限制输入值在有效范围内
    if (value < inMin) value = inMin;
    if (value > inMax) value = inMax;
    
    // 线性映射到 -1 到 1 的范围
    return ((value - inMin) / (inMax - inMin)) * 2.0f - 1.0f;
}

int run(void *dora_context)
{
    unsigned char counter = 0;

    while (true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            if (strncmp("SteeringCmd", data_id, 11) == 0)
            {
                SteeringCmd_h *steering_data = reinterpret_cast<SteeringCmd_h *>(data);
                steering_angle = - steering_data->SteeringAngle;
                steer_update = true;
                std::cout << "[Input] Received SteeringCmd with throttle: " << steering_angle << std::endl;
            }

            if (strncmp("TrqBreCmd", data_id , 9) == 0)
            {
                TrqBreCmd_h *trqbre_data = reinterpret_cast<TrqBreCmd_h *>(data);
                temp.bre_enable = trqbre_data->bre_enable;
                temp.bre_value = trqbre_data->bre_value;
                temp.trq_enable = trqbre_data->trq_enable;
                temp.trq_value = trqbre_data->trq_value_3;
                trq_update = true;
                std::cout << "[Input] Received TrqBreCmd with throttle: " << temp.trq_value << ", brake: " << temp.bre_value << std::endl;
            }

            if(trq_update && steer_update)
            {
                // 转向角度输入是弧度值，最大转向角为30度 = 0.5236弧度
                float minAngleRad = -0.5236f; // -30度转弧度
                float maxAngleRad = 0.5236f;  // +30度转弧度
                // 线性映射到 -1 到 1 的范围，确保充分利用Carla的转向范围
                steering_angle = mapToRange(steering_angle, minAngleRad, maxAngleRad);

                json control_data;
                control_data["id"] = "control";
                control_data["steer"] = steering_angle;
                std::cout << "steer: " << steering_angle<<  std::endl;
                control_data["throttle"] = temp.trq_value / 18;
                control_data["brake"] = temp.bre_value;
                // 将JSON对象转换为字符串
                std::string json_str = control_data.dump();

                // 目标地址和端口
                struct sockaddr_in server_addr;
                memset(&server_addr, 0, sizeof(server_addr));
                server_addr.sin_family = AF_INET;
                server_addr.sin_port = htons(23456);
                if (inet_pton(AF_INET, "192.168.1.101", &server_addr.sin_addr) <= 0) {
                    std::cerr << "Invalid address or address not supported" << std::endl;
                    close(sockfd);
                    return 1;
                }

                ssize_t sent = sendto(sockfd, json_str.c_str(), json_str.length(), 0,
                          (struct sockaddr*)&server_addr, sizeof(server_addr));
                if (sent < 0) {
                    std::cerr << "Failed to send data" << std::endl;
                } else {
                    std::cout << "Sent " << sent << " bytes to server" << std::endl;
                }
                
                // 重置更新标志
                steer_update = false;
                trq_update = false;
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main(){

    std::cout << "HELLO control" << std::endl;
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);
    close(sockfd);
    std::cout << "GOODBYE  control" << std::endl;
    return 0;
}
