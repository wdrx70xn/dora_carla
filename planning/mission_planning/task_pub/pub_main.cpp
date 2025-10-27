extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include <stdlib.h>
#include <mysql.h>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include "datatype.h"
#include "read_mysql_core.h"

#include <RoadAttri.h>
#include <Task.h>
#include <Localization.h>
using namespace std;


struct DBAddr db_addr = {"sde","123456","127.0.0.1","taskdb",5432};   //postgresql
Read_Mysql_Core read_mysql(db_addr);
CurPose_h curpose_msg;
struct RoadAttribute road_atr; 
struct TaskAttribute task_atr; 
Task_h *task = new Task_h;   
int ct = 0;

struct ThreadData 
{
    vector<int> task_execu_flag;
    void* dora_context;
};

void onCurrentPoseMsgRecvd(char *msg)
{
    ct = 1;
    CurPose_h *cur_pose = reinterpret_cast<CurPose_h *>(msg);
    curpose_msg.x = cur_pose->x;
    curpose_msg.y = cur_pose->y;
    curpose_msg.theta = cur_pose->theta;
    curpose_msg.s = cur_pose->s;
    curpose_msg.d = cur_pose->d;
    return ;
}

void GetRoadAtr()
{
   for(int i=0;i<read_mysql.road_atr_vec.size();i++)
	{
        if(read_mysql.road_atr_vec[i].start_s < curpose_msg.s && curpose_msg.s < read_mysql.road_atr_vec[i].end_s)
            {
                road_atr = read_mysql.road_atr_vec[i];
                std::cout << "______________speed: " << road_atr.velocity << std::endl;
            }
    }
	 return ;
}

void GetTaskAtr(std::vector<int>& task_execu_flag, void *dora_context)
{
	for(int i=0;i<read_mysql.task_atr_vec.size();i++)
    {
        
        if(read_mysql.task_atr_vec[i].start_s < curpose_msg.s && curpose_msg.s < read_mysql.task_atr_vec[i].end_s )
        {
            if(!task_execu_flag[i]){
    
                task_atr = read_mysql.task_atr_vec[i];

                task->task_type = task_atr.task_type; 
                std::cout << "task.task_type: " << task_atr.task_type << std::endl;   
                task->s_start = task_atr.start_s;
                task->s_end = task_atr.end_s;
                task->info = task_atr.task_info;
                task->info_2 = 101.5;

                std::string out_id = "task_exc_service";

                Task_h *task_ptr = task;
                char *output_data = (char *)task_ptr;
                size_t output_data_len = sizeof(Task_h);
                std::cout << "output_data_len: " << output_data_len << std::endl;
                std::cout << "***********pub_task***********" << std::endl;
                int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data, output_data_len);
                if(result != 0){
                    std::cerr << "failed to send output" << std::endl;
                }
                std::cout << "***********pub_task_end***********" << std::endl;
            }	
            else ;

        }

        else task_execu_flag[i] = false;
    }

	 return ;
}


void* Pub_road_attri_pthread(void *dora_context)
{
while(true)
{
    if(ct > 0)
    {
        std::cout << "**********Pub_road_attri_pthread**********" << std::endl;
        const int rate = 10;   // 设定频率为 xx HZ
        const chrono::milliseconds interval((int)(1000/rate));
        RoadAttri_h *road_attri_msg = new RoadAttri_h;
        road_attri_msg->velocity = road_atr.velocity;
        std::cout << " road_attri_msg.velocity : " << road_attri_msg->velocity << std::endl;
        road_attri_msg->road_width = road_atr.road_width;

        road_attri_msg->aeb_front = road_atr.aeb_front;
        road_attri_msg->aeb_back = road_atr.aeb_back;
        road_attri_msg->aeb_left = road_atr.aeb_left;
        road_attri_msg->aeb_right = road_atr.aeb_right;

        std::string out_id = "road_attri_msg";

        RoadAttri_h *road_attri_msg_ptr = road_attri_msg;
        char *output_data = (char *)road_attri_msg_ptr;
        size_t output_data_len = sizeof(RoadAttri_h);
        std::cout << "output_data_len: " << output_data_len << std::endl;
        std::cout << "***********road_pub***********" << std::endl;
        int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data, output_data_len);
        if(result != 0){
            std::cerr << "failed to send output" << std::endl;
        }
        std::cout << "***********road_pub_end***********" << std::endl;

        // std::cout << "******************************************" << std::endl;
        delete road_attri_msg;

        this_thread::sleep_for(interval);
    }
}
    return NULL;
}

void* Get_task_pthread(void *arg)
{
    while(true)
    {
        if(ct > 0)
        {
            std::cout << "**********Get_task_pthread**********" << std::endl;
            const int rate = 10;   // 设定频率为 xx HZ
            const chrono::milliseconds interval((int)(1000/rate));

            ThreadData* data = static_cast<ThreadData*>(arg);
            std::vector<int> task_execu_flag = data->task_execu_flag;
            void* dora_context = data->dora_context;

            GetRoadAtr();
            GetTaskAtr(task_execu_flag, dora_context);
            this_thread::sleep_for(interval);
            // std::cout << "******************************************" << std::endl;

        }
    }

    return NULL;
}

void Read_Mysql_Core::Read_Pqdata()
{
  const char* sql;
  try{

    connection conn("dbname="+pq_dbname+" user="+pq_user+" password="+pq_password+" hostaddr="+pq_hostaddr+" port="+pq_port);
	if(conn.is_open()){
		connect_is_ok = true;
      cout<<"connect to postgresql successfully. dbname="<<conn.dbname()<<endl;
    }else{
		connect_is_ok = false;
      cout<<"connect to postgresql fail."<<endl;
      return ;
    }
    nontransaction ntx(conn);
    sql = "select * from attri_tb";
    result r(ntx.exec(sql));
    for(result::const_iterator c=r.begin(); c!=r.end(); ++c){
        road_atr.id_map = c[0].as<int>();
        road_atr.velocity = c[1].as<double>();
        road_atr.road_width = c[2].as<double>();
        road_atr.aeb_front = c[3].as<double>();
	    road_atr.aeb_back = c[4].as<double>();
        road_atr.aeb_left = c[5].as<double>();
	    road_atr.aeb_right = c[6].as<double>();        
		road_atr.start_s = c[7].as<double>();
		road_atr.end_s = c[8].as<double>();
		road_atr_vec.push_back(road_atr);
	//cout<<"_______________________________________"<<endl;
    cout << "road_atr.velocity: " << road_atr.velocity << " road_atr.road_width: " << road_atr.road_width
         << " road_atr.start_s: " << road_atr.start_s << " road_atr.end_s: " << road_atr.end_s << endl;
    } 

    sql = "select * from task_tb";
    result rr(ntx.exec(sql));
    for(result::const_iterator c=rr.begin(); c!=rr.end(); ++c){
        task_atr.id_task = c[0].as<int>();
		task_atr.task_type = c[1].as<int>();
		task_atr.start_s = c[2].as<double>();
		task_atr.end_s = c[3].as<double>();
		task_atr.task_info = c[4].as<double>();
		task_atr.notes = const_cast<char*>( c[5].as<string>().c_str() );    
		task_atr_vec.push_back(task_atr);   
	//cout<<"_______________________________________"<<endl;
    cout << "task_atr.id_task: " << task_atr.id_task << " task_atr.task_type: " << task_atr.task_type << endl;
    } 

    conn.disconnect();
  }catch(const std::exception &e){
	cout<<"postgre connection error!!!!!!!"<<endl;
	connect_is_ok = false;
    cerr<<e.what()<<endl;
    return ;
  }

}


int run(void *dora_context)
{

    while (true)
    {

        void * event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            // std::cout << "2222222222222222222222222222" << std::endl;
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            std::cout << "Input Data length: " << data_len << std::endl;
            if (strcmp("cur_pose_all", data_id) == 0)
            {
                onCurrentPoseMsgRecvd(data);
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


int main()
{
    std::cout << "task_pub" << std::endl;

    auto dora_context = init_dora_context_from_env();


    read_mysql.Read_Pqdata();
    while( !read_mysql.connect_is_ok )
    {
        read_mysql.Read_Pqdata();
    }

    vector<int> task_execu_flag(read_mysql.task_atr_vec.size(), 0);

    ThreadData task_pthread = {task_execu_flag, dora_context};


    pthread_t id_0 = 0;
    pthread_t id_1 = 1;


    if (pthread_create(&id_0, nullptr, Get_task_pthread, &task_pthread) != 0)
    {
        std::cerr << "create Pub_road_attri_msg thread fail!" << std::endl;
        exit(-1);
    }

    if (pthread_create(&id_1, nullptr, Pub_road_attri_pthread, dora_context) != 0)
    {
        std::cerr << "create Pub_road_attri_msg thread fail!" << std::endl;
        exit(-1);
    }

    // std::cout << "111111111111111111111111" << std::endl;

    auto ret = run(dora_context);


    free_dora_context(dora_context);
    
    delete task;

    pthread_cancel(id_0);
    pthread_cancel(id_1);

    std::cout << "END task_pub" << std::endl;

    return ret;
}