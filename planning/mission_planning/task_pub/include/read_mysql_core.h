#ifndef READ_MYSQL_CORE_H
#define READ_MYSQL_CORE_H

#include <stdlib.h>
#include <mysql.h>
#include <string>
#include <string.h>
#include <iostream>
#include<vector>
#include "datatype.h"
#include <pqxx/pqxx>

using namespace std;
using namespace pqxx;

class Read_Mysql_Core
{
public:
    Read_Mysql_Core(DBAddr &db_addr);                 //构造函数
    vector<RoadAttribute> road_atr_vec; //RoadAttribute集合
    vector<TaskAttribute> task_atr_vec; //TaskAttribute集合
    bool connect_is_ok = true;
    bool query_is_ok = true;

    void MysqlInit();
    void Read_Road_Attri();
    void Read_Task_Attri();
    void Free_Mysql();
	void Read_Pqdata();

private:
    struct RoadAttribute road_atr;      //从数据库读取的RoadAttribute
    struct TaskAttribute task_atr;      //从数据库读取的TaskAttribute
	//mysql 配置
	char user[20];
	char pswd[20];
	char host[20];    //mysql服务器地址
	char dbname[20];   //要连接的数据库名称
	unsigned int port ;   

	//postgre配置
	string pq_dbname;
	string pq_user;
	string pq_password;
	string pq_hostaddr;
	string pq_port;

	MYSQL mysql1;
	MYSQL_RES *mysql_result;
	MYSQL_ROW sql_row;
	MYSQL_FIELD *fields;
	int res;

};



Read_Mysql_Core::Read_Mysql_Core(DBAddr &db_addr)
{
	strcpy(user,db_addr.user);
	strcpy(pswd,db_addr.pswd);
	strcpy(host,db_addr.host);    
	strcpy(dbname,db_addr.dbname);  
	port = db_addr.port; 

	pq_dbname = db_addr.dbname;
	pq_user = db_addr.user;
	pq_password = db_addr.pswd;
	pq_hostaddr = db_addr.host;
	pq_port = to_string(db_addr.port); 
}



#endif // READ_MYSQL_CORE_H