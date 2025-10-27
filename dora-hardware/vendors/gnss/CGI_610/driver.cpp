extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <time.h>
#include <mutex>
#include "uart_api.h"
#include "iostream"
#include "string"
#include "cstring"
#include "imu_msg.h"
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry> // 包含四元数相关功能

#include "Localization.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;

string gnss_id = "gnss";
//用于保存解析后数据
gpgga gpgga_msg;
gpvtg gpvtg_msg;
pashr pashr_msg;
gpchc gpchc_msg;
gpfpd gpfpd_msg;

void pub_gnss(void* dora_context)
{

    const int rate = 50 ;   // 设定频率为 xx HZ
    const chrono::milliseconds interval_phread((int)(1000/rate));
    while(1)
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        json j;
        j["frame_id"] = gnss_id;
        j["seq"] = 1;
        j["sec"] =  tv.tv_sec;
        j["nanosec"] = tv.tv_usec * 1000;
        
        // j["latitude"] = gpfpd_msg.Lattitude;
        // j["longitude"] = gpfpd_msg.Longitude;
        // j["altitude"] = gpfpd_msg.Altitude;
        // j["status"] = gpfpd_msg.Status;

        j["latitude"] = gpchc_msg.Lattitude;
        j["longitude"] = gpchc_msg.Longitude;
        j["altitude"] = gpchc_msg.Altitude;
        j["status"] = gpchc_msg.Status;
        
        // std::cout << "gpchc_msg.Lattitude  " << gpchc_msg.Lattitude << std::endl;
        // std::cout << "gpchc_msg.Longitude  " << gpchc_msg.Longitude << std::endl;
        

        std::string json_string = j.dump(4); // 参数 4 表示缩进宽度
        // std::cout << "111111111 " << std::endl;
        // std::cout << "json: " << json_string << std::endl;
        char *c_json_string = new char[json_string.length() + 1];
        strcpy(c_json_string, json_string.c_str());
        std::string out_id = "DoraNavSatFix";
        // std::cout<<json_string<<endl;
        int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
        if (result != 0)
        {
            std::cerr << "failed to send output" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        json j_1;
        j_1["seq"] = 1;
        j_1["sec"] =  tv.tv_sec;
        j_1["nanosec"] = tv.tv_usec * 1000;


        // double roll = gpfpd_msg.Roll * M_PI / 180.0;
        // double pitch = gpfpd_msg.Pitch * M_PI / 180.0;
        // double heading = gpfpd_msg.Heading * M_PI / 180.0;

        double roll = gpchc_msg.Roll * M_PI / 180.0;
        double pitch = gpchc_msg.Pitch * M_PI / 180.0;
        double heading = gpchc_msg.Heading * M_PI / 180.0;


        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        
        j_1["x"] = q.x();
        j_1["y"] = q.y();
        j_1["z"] = q.z();
        j_1["w"] = q.w();

        j_1["Roll"] = gpchc_msg.Roll;
        j_1["Pitch"] = gpchc_msg.Pitch;
        j_1["Heading"] = gpchc_msg.Heading;

        // j_1["Roll"] = gpfpd_msg.Roll;
        // j_1["Pitch"] = gpfpd_msg.Pitch;
        // j_1["Heading"] = gpfpd_msg.Heading;

        std::string json_string_1 = j_1.dump(4); // 参数 4 表示缩进宽度
        // std::cout << "111111111 " << std::endl;
        // std::cout << "json: " << json_string << std::endl;
        char *c_json_string_1 = new char[json_string_1.length() + 1];
        strcpy(c_json_string_1, json_string_1.c_str());
        std::string out_id_1 = "DoraQuaternionStamped";
        // std::cout<<json_string<<endl;
        int result_1 = dora_send_output(dora_context, &out_id_1[0], out_id_1.length(), c_json_string_1, std::strlen(c_json_string_1));
        if (result_1 != 0)
        {
            std::cerr << "failed to send output" << std::endl;
        }


        this_thread::sleep_for(interval_phread);
    }   

    return ;     
}

int main()
{
    //std::cout << "pub gnss for dora " << std::endl;
    auto dora_context = init_dora_context_from_env();
    // ros::Publisher navi_data_pub = nh.advertise<custom_msgs::NaviData>("navi_msg", 1);
    // ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("raw_imu", 1);
    NaviData_h navi_msg;
    std::string recv_string;
    std::string gpg_msg;
    std::string header;
    std::string uart_num = "/dev/ttyUSB0";

    int start_position = 0;
    int back_position = 0;         //记录$位置
    int comma_position_front = 0;
    int comma_position_back = 0;   //记录，位置

    int index = 0;
    ssize_t ret = 0;
    int read_num = 0;

    char msg_buff[512];
    int uart = uart_open(uart_num.c_str());
    fcntl(uart,F_SETFL,0);  //阻塞
    uart_conf_set(uart,460800,8,1,'N');
    memset(msg_buff,0,512);

    // pthread_t id = 0;
    // // 开启车辆状态线程
    // if (pthread_create(&id, nullptr, pub_gnss, dora_context) != 0)
    // {
    //     std::cerr << "create pub_gnss_pthread thread fail!" << std::endl;
    //     exit(-1);
    // }
    std::thread t(pub_gnss, dora_context);
    t.detach();

    // std::cout << "6666666" << std::endl;

    const int rate = 50;   // 设定频率为 xx HZ
    const chrono::milliseconds interval((int)(1000/rate));
    while (true)
    {
        for (read_num=0 ; read_num<200; read_num+=ret) 
        { //强制读取500字符
	 	    ret = read(uart,msg_buff+read_num,200-read_num);
	    }
        recv_string = msg_buff;
        // std::cout << "rec : " << recv_string <<std::endl;
        if (recv_string.empty()) 
        {
            std::cout << " gnss no data!!! " << std::endl;
            return -1;
        }
        start_position = 0;
        back_position = 0;
     
        while(true)
        {
            if((start_position = recv_string.find("$",back_position)) == std::string::npos)
                break;
            if((back_position = recv_string.find("$",start_position+1)) == std::string::npos)
                break;
            gpg_msg = recv_string.substr(start_position+1,back_position-start_position-2);//截取$之间的字符串
            header = recv_string.substr(start_position+1,5);

            //std::cout<< gpg_msg <<std::endl;

            // struct timeval tv;
            // gettimeofday(&tv, NULL);//获取时间
            // double time = tv.tv_sec + tv.tv_usec * 1e-6;
            // std::cout << std::fixed << std::setprecision(6)<< "time: " << time << std::endl;

            comma_position_front = 0;
            comma_position_back = 0;
            index = 0;
            // std::cout << " herder " << header.c_str() << std::endl;

           if(!strncmp(header.c_str(),"GPFPD",5)){

                strcpy(gpgga_msg.header,header.c_str());

                while(1)
                {
                    if((comma_position_front = gpg_msg.find(",",comma_position_back)) == std::string::npos)
                        break;
                    if((comma_position_back = gpg_msg.find(",",comma_position_front+1)) == std::string::npos)
                        break;

                    index++;



                    
                    switch (index)
                    { //现只填充了需要的数据
                        case 3:
                            gpfpd_msg.Heading = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                            // std::cout << " gpfpd_msg.Heading : " << gpfpd_msg.Heading << std::endl;
                            break;
                        case 4:
                            gpfpd_msg.Pitch = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                            // std::cout << "pitch: " << gpfpd_msg.Pitch << std::endl;
                            break;
                        case 5:
                            gpfpd_msg.Roll = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                            break;
                        case 6:	
                            gpfpd_msg.Lattitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                            // std::cout << std::fixed << std::setprecision(7) << "gpgga_msg.lat: " << gpfpd_msg.Lattitude << std::endl;
                            break;
                        case 7:
                            gpfpd_msg.Longitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str()); 
                            // std::cout << std::fixed << std::setprecision(7) << "gpgga_msg.lon: " << gpfpd_msg.Longitude << std::endl;
                            break;
                        case 8:
                            gpfpd_msg.Altitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                            break;
                        case 15:
                            gpfpd_msg.Status = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                            break;

                        default: 
                            break;    
                    }     
                }
            }


                        // //解析GPCHC消息
            // if(!strncmp(header.c_str(),"GPGGA",5)){

            //     strcpy(gpgga_msg.header,header.c_str());

            //     while(1)
            //     {
            //         if((comma_position_front = gpg_msg.find(",",comma_position_back)) == std::string::npos)
            //             break;
            //         if((comma_position_back = gpg_msg.find(",",comma_position_front+1)) == std::string::npos)
            //             break;

            //         index++;

            //         switch (index)
            //         { //现只填充了需要的数据
            //         case 2:	

			//             gpgga_msg.lat = floor(atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str()) *0.01) + (atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str()) *0.01-floor(atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str()) *0.01))/0.6 ;
            //             std::cout << std::fixed << std::setprecision(7) << "gpgga_msg.lat" << gpgga_msg.lat << std::endl;
            //             break;
            //         case 4:

            //             gpgga_msg.lon = floor(atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str()) *0.01) + (atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str()) *0.01-floor(atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str()) *0.01))/0.6; 
            //             std::cout << std::fixed << std::setprecision(7) << "gpgga_msg.lon" << gpgga_msg.lon << std::endl;
            //             break;
            //         case 6:
            //             gpgga_msg.qual = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
            //             break;
            //         case 9:
            //             gpgga_msg.alt = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
            //             break;
            //         default: 
            //             break;    
            //         }     
            //     }
            // }



            if(!strncmp(header.c_str(),"GPCHC",5)){

                strcpy(gpchc_msg.Header,header.c_str());

                while(1){
                    if((comma_position_front = gpg_msg.find(",",comma_position_back)) == std::string::npos)
                        break;
                    if((comma_position_back = gpg_msg.find(",",comma_position_front+1)) == std::string::npos)
                        break;
                    
                    index++;

                    switch (index){ 
                    case 1:
                        gpchc_msg.GPSWeek = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 2:
                        gpchc_msg.GPSTime = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 3:
                        gpchc_msg.Heading = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        // std::cout << "gpchc_msg.Heading: " << gpchc_msg.Heading << std::endl;
                        break;
                    case 4:
                        gpchc_msg.Pitch = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 5:
                        gpchc_msg.Roll = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 6:
                        gpchc_msg.gyro_x = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 7:
                        gpchc_msg.gyro_y = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 8:
                        gpchc_msg.gyro_z = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 9:
                        gpchc_msg.acc_x = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 10:
                        gpchc_msg.acc_y = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 11:
                        gpchc_msg.acc_z = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 12:
                        gpchc_msg.Lattitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        // std::cout << "lat " << gpchc_msg.Lattitude << std::endl;
                        break;
                    case 13:
                        gpchc_msg.Longitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 14:
                        gpchc_msg.Altitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 15:
                        gpchc_msg.Ve = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 16:
                        gpchc_msg.Vn = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 17:
                        gpchc_msg.Vu = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 18:
                        gpchc_msg.V = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 19:
                        gpchc_msg.NSV1 = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 20:
                        gpchc_msg.NSV2 = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 21:
                        gpchc_msg.Status = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 22:
                        gpchc_msg.Age = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    default: 
                        break;    
                    }     
                }

                 //std::cout<< "gpvtg_msg.Speed_over_ground_km:" << gpvtg_msg.Speed_over_ground_km <<std::endl;
            }

            
        }

        this_thread::sleep_for(interval);
    }
    
    free_dora_context(dora_context);
    //std::cout << "exit pub gnss node ..." << std::endl;
    
    return 0;
}
