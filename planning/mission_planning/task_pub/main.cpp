extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <chrono>
#include "datatype.h"
#include "RoadAttri.h"
using namespace std;



void readFileAndParseData(const std::string& filename, RoadAttribute& roadAttr, void* dora_context) {
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Unable to open file" << std::endl;
        return;
    }

    std::unordered_map<std::string, float> data;
    std::string line;

    // 逐行读取文件
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        float value;

        // 将行数据解析为键值对
        if (iss >> key >> value) {
            data[key] = value; // 将键值对存入unordered_map
        } 
    }

    file.close();

    if (data.find("id") != data.end()) roadAttr.id_map = static_cast<int>(data["id"]);
    if (data.find("velocity") != data.end()) roadAttr.velocity = data["velocity"];
    if (data.find("road_width") != data.end()) roadAttr.road_width = data["road_width"];
    if (data.find("aeb_front") != data.end()) roadAttr.aeb_front = data["aeb_front"];
    if (data.find("aeb_back") != data.end()) roadAttr.aeb_back = data["aeb_back"];
    if (data.find("aeb_left") != data.end()) roadAttr.aeb_left = data["aeb_left"];
    if (data.find("aeb_right") != data.end()) roadAttr.aeb_right = data["aeb_right"];
    if (data.find("start_s") != data.end()) roadAttr.start_s = data["start_s"];
    if (data.find("end_s") != data.end()) roadAttr.end_s = data["end_s"];


    RoadAttri_h *road_attri_msg = new RoadAttri_h;
    road_attri_msg->velocity = roadAttr.velocity;
    std::cout << "road_attri_msg->velocity: " << road_attri_msg->velocity << std::endl;
    road_attri_msg->road_width = roadAttr.road_width;
    road_attri_msg->aeb_front = roadAttr.aeb_front;
    road_attri_msg->aeb_back = roadAttr.aeb_back;
    road_attri_msg->aeb_left = roadAttr.aeb_left;
    road_attri_msg->aeb_right = roadAttr.aeb_right;


    std::string out_id = "road_attri_msg";
    RoadAttri_h *road_attri_msg_ptr = road_attri_msg;
    char *output_data = (char *)road_attri_msg_ptr;
    size_t output_data_len = sizeof(RoadAttri_h);
    // std::cout << "output_data_len: " << output_data_len << std::endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data, output_data_len);
    if(result != 0){
        std::cerr << "failed to send output" << std::endl;
    }

    // std::cout << "******************************************" << std::endl;
    delete road_attri_msg;

}



int run(void *dora_context)
{

    while(true)
    {
        const int rate = 10;   // 设定频率为 xx HZ
        const chrono::milliseconds interval((int)(1000/rate));
        void * event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            RoadAttribute roadAttr;
            std::string filename = "/home/nano/disk/dora_project/hongmeng/road_msg.txt";
            readFileAndParseData(filename, roadAttr, dora_context);
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
        this_thread::sleep_for(interval);

    }
    return 0;
}


int main()
{
    std::cout << "task_pub" << std::endl;

    auto dora_context = init_dora_context_from_env();

    auto ret = run(dora_context);


    free_dora_context(dora_context);
    

    std::cout << "END task_pub" << std::endl;

    return ret;
}