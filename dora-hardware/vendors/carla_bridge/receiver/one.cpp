extern "C" {
    #include "node_api.h"
    #include "operator_api.h"
    #include "operator_types.h"
}
#include <iostream>
#include <vector>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <sys/time.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

// rs lidar driver
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/common/error_code.hpp>
#include <rs_driver/utility/sync_queue.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

using json = nlohmann::json;
std::mutex cout_mutex;

// 安全打印函数，使用互斥锁保护 std::cout
void safe_print(const std::string& message) {
    std::lock_guard<std::mutex> lock(cout_mutex);
    std::cout << message << std::endl;
}

// 定义结构体来存储 DoraNavSatFix 数据（简化版，根据实际情况调整）
struct DoraNavSatFix {
    std::string frame_id;  // 添加 frame_id 成员
    double x;
    double y;
    double z;
};

// 定义结构体来存储 IMU 数据
struct DoraIMU {
    struct {
        double x;
        double y;
        double z;
    } accelerometer;
    struct {
        double x;
        double y;
        double z;
    } gyroscope;
    double heading_deg;
};

typedef PointXYZI PointT;                       // x,y,z, intensity;
                                                // intensity: ji guang fan she qiang du
                                                // this is a point in the point cloud

/// @brief PointCloudMsg
//  typedef std::vector<PointT> VectorT
//  uint32_t height = 0
//  uint32_t width = 0
//  bool is_dense = false
//  double timestamp = 0
//  uint32_t seq = 0  //sequence number of message
//  std::string frame_id = ""
//  vectorT points
typedef PointCloudT<PointT> PointCloudMsg;

// 暂时注释掉命名空间使用，避免找不到命名空间的错误
// using namespace robosense::lidar;
using robosense::lidar::SyncQueue;
// using rs_driver::common::Error;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

// 从free队列里面取
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
    // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
    //       so please DO NOT do time-consuming task here.
    std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
    if (msg.get() != nullptr) // 使用 nullptr 替代 NULL
    {
        return msg;
    }

    return std::make_shared<PointCloudMsg>();
}

// 填到一个新的队列里
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
    // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
    //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
    stuffed_cloud_queue.push(msg);
    safe_print("Received a stuffed point cloud message with " + std::to_string(msg->points.size()) + " points and pushed it to the queue.");
}

// 提取 IMU 数据
void extractIMUData(const json& j, DoraIMU& imu) {
    if (j.contains("accelerometer") && j["accelerometer"].is_object()) {
        const auto& accel = j["accelerometer"];
        if (accel.contains("x") && accel["x"].is_number()) {
            imu.accelerometer.x = accel["x"].get<double>();
        }
        if (accel.contains("y") && accel["y"].is_number()) {
            imu.accelerometer.y = accel["y"].get<double>();
        }
        if (accel.contains("z") && accel["z"].is_number()) {
            imu.accelerometer.z = accel["z"].get<double>();
        }
    }
    if (j.contains("gyroscope") && j["gyroscope"].is_object()) {
        const auto& gyro = j["gyroscope"];
        if (gyro.contains("x") && gyro["x"].is_number()) {
            imu.gyroscope.x = gyro["x"].get<double>();
        }
        if (gyro.contains("y") && gyro["y"].is_number()) {
            imu.gyroscope.y = gyro["y"].get<double>();
        }
        if (gyro.contains("z") && gyro["z"].is_number()) {
            imu.gyroscope.z = gyro["z"].get<double>();
        }
    }
    if (j.contains("heading_deg") && j["heading_deg"].is_number()) {
        imu.heading_deg = j["heading_deg"].get<double>();
    }
}

// 提取 GNSS 数据
void extractGNSSData(const json& j, DoraNavSatFix& navSatFix) {
    if (j.contains("frame_id")) {
        navSatFix.frame_id = j["frame_id"].get<std::string>();
    }
    if (j.contains("x") && j["x"].is_number()) {
        navSatFix.x = j["x"].get<double>();
    }
    if (j.contains("y") && j["y"].is_number()) {
        navSatFix.y = j["y"].get<double>();
    }
    if (j.contains("z") && j["z"].is_number()) {
        navSatFix.z = j["z"].get<double>();
    }
}

// 发送 IMU 数据
void sendIMUData(const DoraIMU& imu, void* dora_context) {
    json msg;
    msg["id"] = "imu";
    msg["accelerometer"]["x"] = imu.accelerometer.x;
    msg["accelerometer"]["y"] = imu.accelerometer.y;
    msg["accelerometer"]["z"] = imu.accelerometer.z;
    msg["gyroscope"]["x"] = imu.gyroscope.x;
    msg["gyroscope"]["y"] = imu.gyroscope.y;
    msg["gyroscope"]["z"] = imu.gyroscope.z;
    msg["heading_deg"] = std::round(imu.heading_deg * 100) / 100.0;  // 保留两位小数

    std::string json_string_imu = msg.dump(4);
    char *c_json_string_imu = new char[json_string_imu.length() + 1];
    strcpy(c_json_string_imu, json_string_imu.c_str());

    std::string out_id_imu = "DoraQuaternionStamped";

    safe_print("Sending IMU data: " + json_string_imu);

    int result_imu = dora_send_output(dora_context, &out_id_imu[0], out_id_imu.length(), c_json_string_imu, std::strlen(c_json_string_imu));

    if (result_imu != 0) {
        safe_print("failed to send output");
    } else {
        safe_print("IMU data sent successfully.");
    }
    // 释放内存
    delete[] c_json_string_imu;
}

// 发送 GNSS 数据
void sendGNSSData(const DoraNavSatFix& navSatFix, void* dora_context) {
    json msg;
    msg["id"] = "gnss";
    msg["latitude"] = navSatFix.x;
    msg["longitude"] = navSatFix.y;
    msg["altitude"] = navSatFix.z;
    msg["frame_id"] = navSatFix.frame_id;  // 添加 frame_id 字段

    std::string json_string = msg.dump(4);
    char *c_json_string = new char[json_string.length() + 1];
    strcpy(c_json_string, json_string.c_str());

    std::string out_id = "DoraNavSatFix";

    safe_print("Sending GNSS data: " + json_string);

    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
    if (result != 0) {
        safe_print("failed to send GNSS output");
    } else {
        safe_print("GNSS data sent successfully.");
    }
    // 释放内存
    delete[] c_json_string;
}

// UDP 接收函数
void receiveUDP(int port, const char* interface_ip, void* dora_context) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        safe_print("Socket creation failed: " + std::string(strerror(errno)));
        return;
    }
    safe_print("Socket created successfully.");

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    if (interface_ip != nullptr) {
        if (inet_pton(AF_INET, interface_ip, &server_addr.sin_addr) <= 0) {
            safe_print("Invalid IP address: " + std::string(strerror(errno)));
            close(sock);
            return;
        }
    } else {
        server_addr.sin_addr.s_addr = INADDR_ANY;
    }
    server_addr.sin_port = htons(port);

    if (bind(sock, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        safe_print("Bind failed: " + std::string(strerror(errno)));
        close(sock);
        return;
    }
    safe_print("Socket bound to port " + std::to_string(port) + " successfully.");
    if (interface_ip != nullptr) {
        safe_print("Listening on interface IP: " + std::string(interface_ip));
    }

    while (true) {
        char buffer[4096];
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        ssize_t received_bytes = recvfrom(sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (received_bytes < 0) {
            safe_print("Receive failed: " + std::string(strerror(errno)) + ", Error code: " + std::to_string(errno));
            continue;
        }

        safe_print("Received " + std::to_string(received_bytes) + " bytes of data.");
        std::string data_str(buffer, received_bytes);
        safe_print("Received data: " + data_str);

        try {
            json j = json::parse(data_str);
            safe_print("JSON data parsed successfully.");

            if (j.contains("id")) {
                std::string id = j["id"].get<std::string>();
                if (id == "imu") {
                    DoraIMU imu;
                    extractIMUData(j, imu);
                    safe_print("Received IMU data: "
                               "Accel: " + std::to_string(imu.accelerometer.x) + ", " + std::to_string(imu.accelerometer.y) + ", " + std::to_string(imu.accelerometer.z)
                               + ", Gyro: " + std::to_string(imu.gyroscope.x) + ", " + std::to_string(imu.gyroscope.y) + ", " + std::to_string(imu.gyroscope.z)
                               + ", Heading: " + std::to_string(imu.heading_deg));
                    sendIMUData(imu, dora_context);
                } else if (id == "gnss") {
                    DoraNavSatFix navSatFix;
                    extractGNSSData(j, navSatFix);
                    safe_print("Received GNSS data: "
                               "Frame ID: " + navSatFix.frame_id
                               + ", x: " + std::to_string(navSatFix.x) + ", "
                               + "y: " + std::to_string(navSatFix.y) + ", "
                               + "z: " + std::to_string(navSatFix.z));
                    sendGNSSData(navSatFix, dora_context);
                }
            }
        } catch (const json::parse_error& e) {
            safe_print("JSON 解析错误: " + std::string(e.what()));
        } catch (const json::type_error& e) {
            safe_print("JSON 类型错误: " + std::string(e.what()));
        }
    }

    close(sock);
    safe_print("Socket closed.");
}

// TCP 接收函数
void receiveTCP(int port)
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // 创建套接字文件描述符
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        safe_print("socket failed: " + std::string(strerror(errno)));
        return;
    }

    // 设置套接字选项
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        safe_print("setsockopt: " + std::string(strerror(errno)));
        return;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // 绑定套接字到指定地址和端口
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        safe_print("bind failed: " + std::string(strerror(errno)));
        return;
    }
    // 监听连接
    if (listen(server_fd, 3) < 0)
    {
        safe_print("listen: " + std::string(strerror(errno)));
        return;
    }

    safe_print("TCP receiver started, listening on port " + std::to_string(port));

    while (true)
    {
        safe_print("Waiting for a new connection...");
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)
        {
            if (errno == EINTR) {
                safe_print("accept interrupted by signal, retrying...");
                continue;
            }
            safe_print("accept: " + std::string(strerror(errno)));
            continue;
        }
        safe_print("New connection accepted.");

        // 接收数据长度头部
        uint32_t data_length;
        ssize_t bytes_received = recv(new_socket, &data_length, sizeof(uint32_t), 0);
        if (bytes_received != sizeof(uint32_t))
        {
            if (bytes_received == 0) {
                safe_print("Connection closed by peer while receiving data length header.");
            } else if (bytes_received < 0) {
                safe_print("Error receiving data length header: " + std::string(strerror(errno)));
            }
            close(new_socket);
            continue;
        }

        // 将网络字节序转换为主机字节序
        data_length = ntohl(data_length);

        // 输出接收到的长度值，用于调试
        safe_print("Received data length: " + std::to_string(data_length));

        // 验证数据长度
        if (data_length > 1024 * 1024 * 20) { // 假设最大数据长度为 1MB
            safe_print("Received data length is too large: " + std::to_string(data_length));
            close(new_socket);
            continue;
        }

        // 动态分配缓冲区
        std::vector<char> buffer(data_length);

        // 接收数据
        ssize_t total_received = 0;
        while (total_received < data_length)
        {
            bytes_received = recv(new_socket, buffer.data() + total_received, data_length - total_received, 0);
            if (bytes_received == 0) {
                safe_print("Connection closed by peer while receiving data.");
                break;
            } else if (bytes_received < 0) {
                if (errno == EINTR) {
                    continue;
                }
                safe_print("Error receiving data: " + std::string(strerror(errno)));
                break;
            }
            total_received += bytes_received;
        }

        if (total_received == data_length)
        {
            std::shared_ptr<PointCloudMsg> msg = std::make_shared<PointCloudMsg>();
            // 调整数据解析逻辑
            size_t num_points = data_length / (3 * sizeof(int16_t)); // 数据是 xyz 三个 int16_t
            msg->points.resize(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                if (i * 3 * sizeof(int16_t) + 3 * sizeof(int16_t) > data_length) {
                    safe_print("Data parsing error: insufficient data for point " + std::to_string(i));
                    break;
                }
                int16_t* data = reinterpret_cast<int16_t*>(buffer.data() + i * 3 * sizeof(int16_t));
                msg->points[i].x = static_cast<float>(data[0]) / 100.0f;
                msg->points[i].y = static_cast<float>(data[1]) / 100.0f;
                msg->points[i].z = static_cast<float>(data[2]) / 100.0f;
                msg->points[i].intensity = 0; // 发送端未发送 intensity，这里设为 0
            }

            safe_print("Parsed point cloud with " + std::to_string(msg->points.size()) + " points.");
            driverReturnPointCloudToCallerCallback(msg);
        }

        close(new_socket);
    }

    close(server_fd);
    safe_print("Socket closed.");
}


bool to_exit_process = false;

void processCloud(void)
{
    while (!to_exit_process)
    {
        std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.pop();
        if (msg.get() == nullptr)
        {
            continue;
        }

        safe_print("Processing a point cloud message with " + std::to_string(msg->points.size()) + " points.");

        // Well, it is time to process the point cloud msg, even it is time-consuming.
        // RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

#if 0
        for (auto it = msg->points.begin(); it != msg->points.end(); it++)
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")"
                      << std::endl;
        }
#endif

        free_cloud_queue.push(msg); //这里是说，上面那个#if里面的东西已经把这个点云处理完了，东西都取出来了，那这个点云实例（占内存的）我们就可以重复利用了，就空闲了，把它放入待使用区（free区）
    }
}

int run(void *dora_context)
{
    unsigned char counter = 0;

    to_exit_process = false;
    while (1)
    {
        void *event = dora_next_event(dora_context);
        if (event == nullptr) // 使用 nullptr 替代 NULL
        {
            safe_print("[c node] ERROR: unexpected end of event");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            // copy from rslidar driver
            #if 1
            Vec_uint8_t result;

            std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.pop();
            if (msg.get() == nullptr) // 使用 nullptr 替代 NULL
            {
                continue;
            }

            safe_print("Processing input event, point cloud size: " + std::to_string(msg->points.size()));

            // 假设 RS_MSG 和 RS_REND 是自定义的输出宏，这里先注释掉，避免未定义错误
            // RS_MSG << "tick " << (int)counter << ", remain: " << stuffed_cloud_queue.size() << RS_REND;

            // Well, it is time to process the point cloud msg, even it is time-consuming.
            // RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

            #if 0
            for (auto it = msg->points.begin(); it != msg->points.end(); it++)
            {
                std::cout << std::fixed << std::setprecision(3)
                          << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")"
                          << std::endl;
            }
            #endif

            // free_cloud_queue.push(msg);//这里是说，上面那个#if里面的东西已经把这个点云处理完了，东西都取出来了，那这个点云实例（占内存的）我们就可以重复利用了，就空闲了，把它放入待使用区（free区）

            if (sizeof(PointT) <= 16)
            {
                // RS_MSG << "sizeof(PointT) <= 16 " << RS_REND;
                size_t cloudSize = (((msg->points.size()) + 1) * 16);  // 4byte for message seq, 4bytes empty, 8byte for timestamp,
                                                                        // others for points
                u_int8_t *bytePointCloud = (u_int8_t *)(new PointT[cloudSize / sizeof(PointT)]);

                u_int32_t *seq = (u_int32_t *)bytePointCloud;
                *seq = msg->seq;
                double *timestamp = (double *)(bytePointCloud + 8);
                *timestamp = msg->timestamp;
                // PointT* point = (PointT*)(bytePointCloud + 16);
                // std::vector<PointT>::iterator pointPtr = msg->points.begin();
                // for (int i = 0; i < msg->points.size(); ++i){
                //   *point++ = pointPtr[i];
                // }
                memcpy(bytePointCloud + 16, &(msg->points[0]), cloudSize - 16);

                free_cloud_queue.push(msg);

                result.ptr = bytePointCloud;
                result.len = cloudSize;
                result.cap = cloudSize;
                //return result;
            }
            else if (sizeof(PointT) == 24)
            {                                   // just write them here, I didn't test it
                size_t cloudSize =
                    ((msg->points.size()) * 24);  // 24 bytes for each point, 4*3 bytes for coordinates, 1 byte for intensity, 1
                                                // byte because of byte aligned 2 bytes for rings, 8 bytes for timestamp

                u_int8_t *bytePointCloud = (u_int8_t *)new PointT[cloudSize / sizeof(PointT)];
                memcpy(bytePointCloud, &(msg->points[0]), cloudSize);
                // PointT* point = (PointT*)(bytePointCloud);
                // std::vector<PointT>::iterator pointPtr = msg->points.begin();
                // for (int i = 0; i < msg->points.size(); ++i)
                // {
                //   *(point++) = pointPtr[i];
                // }

                free_cloud_queue.push(msg);
                //Vec_uint8_t result;
                result.ptr = bytePointCloud;
                result.len = cloudSize;
                result.cap = cloudSize;
                //return result;
            }
            else
            {
                safe_print("point size error! This may happen when your system is not byte aligned!");
                result = {.ptr = nullptr}; // 使用 nullptr 替代 NULL
                result.len = 0;
                result.cap = 0;
                //return result;
            }

            #endif

            char *output_data = (char *)result.ptr;
            size_t output_data_len = result.len;
            counter += 1;

            std::string out_id = "pointcloud";
            int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

            //    std::cout
            //         << "dora_send_output: out_id "<<out_id<< "  out_data_len: "<<output_data_len<<std::endl;

            if (resultend != 0)
            {
                safe_print("failed to send output");
                return 1;
            }
            safe_print("Output sent successfully with id: " + out_id + ", length: " + std::to_string(output_data_len) + ", point cloud size: " + std::to_string(msg->points.size()));
        }
        else if (ty == DoraEventType_Stop)
        {
            safe_print("[c node] received stop event");
            to_exit_process = true;
            break;
        }
        else
        {
            safe_print("[c node] received unexpected event: " + std::to_string(ty));
        }

        free_dora_event(event);
    }
    return 0;
}

int main(int argc, char* argv[]) {
    int tcp_port = 5005;
    int udp_port = 12345; // 替换为你实际使用的端口
    const char* interface_ip = nullptr;
    if (argc > 1) {
        interface_ip = argv[1];
    }
    safe_print("rslidar driver for dora (TCP and UDP version) ");
    safe_print("Starting UDP receiver on port " + std::to_string(udp_port));
    if (interface_ip != nullptr) {
        safe_print("Using interface IP: " + std::string(interface_ip));
    }

    auto dora_context = init_dora_context_from_env(); // 假设 init_dora_context_from_env 是可用的初始化函数
    if (!dora_context) {
        safe_print("Failed to initialize dora context.");
        return 1;
    }
    safe_print("Dora context initialized successfully.");

    // 启动 TCP 接收线程
    std::thread tcp_thread(receiveTCP, tcp_port);

    // 启动 UDP 接收线程
    std::thread udp_thread(receiveUDP, udp_port, interface_ip, dora_context);

    // 运行主逻辑
    auto ret = run(dora_context);

    // 等待 UDP 线程结束
    udp_thread.join();

    // 等待 TCP 线程结束
    tcp_thread.join();

    free_dora_context(dora_context);
    safe_print("Program terminated.");
    return ret;
}    