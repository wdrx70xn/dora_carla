#ifndef IMU_MSG_H
#define IMU_MSG_H

#include <stdint.h>

struct gpchc{
    char Header[7];      //0 GPCHC 协议头
    double GPSWeek;      //1 自 1980-1-6 至当前的星期数
    double GPSTime;      //2 自本周日 0:00:00 至当前的秒数
    double Heading;      //3 偏航角
    double Pitch;        //4 俯仰角(-90 至 90)
    double Roll;         //5 横滚角(-180 至 180)
    double gyro_x;       //6 陀螺 X 轴
    double gyro_y;       //7 陀螺 Y 轴
    double gyro_z;       //8 陀螺 Z 轴
    double acc_x;        //9 加表 X 轴
    double acc_y;        //10 加表 Y 轴
    double acc_z;        //11 加表 Z 轴
    double Lattitude;     //12 纬度(-90 至 90)
    double Longitude;     //13 经度(-180 至 180)
    double Altitude;      //14 高度,单位(米)
    double Ve;            //15 东向速度,单位(米/秒)
    double Vn;            //16 北向速度,单位(米/秒)
    double Vu;            //17 天向速度,单位(米/秒)
    double V;             //18 车辆速度,单位(米/秒)
    double NSV1;          //19 主天线 1 卫星数
    double NSV2;          //20 副天线 2 卫星数
    double Status;        //21
    double Age;           //22 差分延时
    char Warming;         //23
    char Cs[5];           //24 校验
};


struct gpgga {
    char header[7];    // 语句的开始，固定为 "$GPGGA"
    double utc;        // UTC 时间，格式为 hhmmss.sss
    double lat;        // 纬度，格式为 ddmm.mmmmm
    char lat_dir;      // 纬度方向，'N' 或 'S'
    double lon;        // 经度，格式为 dddmm.mmmmm
    char lon_dir;      // 经度方向，'E' 或 'W'
    int qual;          // GPS 质量指示
    int sat;           // 正在使用的卫星数量
    double hdop;       // 水平精度
    double alt;        // 海拔高度
    char alt_unit;     // 海拔高度单位
    double Undulation; // 高程异常值
    char U_units;      // 高程异常值单位
    uint8_t age;       // 差分数据龄期
    uint8_t Stn_ID;    // 基准站 ID 号
    char ff[4];        // 校验和
};

struct gpvtg{
    char header[8];       //0 LOG 数据头
    double track_true;    //1 以真北为参考基准的航向
    char True;            //2 真北参考系
    double Track_Mag;     //3 以磁北为参考基准的航向
    char M;               //4 磁北参考系
    double Speed_over_ground;    //5 地面（水平）速度，单位节
    char N;                      //6 单位为节 knots
    double Speed_over_ground_km; //7 地面速度，单位 km/hr
    char K;                      //8 单位为 Km/hr
    char Mode_Ind;               //9 模式指示 A=自主定位 D=差分 E=估算 N=数据无效
    char ff[8];                  //10 校验和
};

struct pashr{
    char header[8];       //0 LOG 数据头
    double UTC_Time;      //1 Utc 时间（hhmmss.ss）
    double Heading;       //2 真北航向（°）
    double Roll;          //3 横滚
    double Pitch;         //4 俯仰
    double Heave;         //5 瞬时起伏
    double Roll_Accuracy;     //6 自评估横滚精度
    double Pitch_Accuracy;    //7 自评估俯仰精度
    double Heading_Accuracy;  //8 自评估航向精度
    uint8_t GPS_Update_Quality_Flag; //9
    uint8_t INS_Status_Flag;         //10
    char ff[8];                      //11 校验和
};


struct gpfpd{
    double Heading;      //3 偏航角
    double Pitch;        //4 俯仰角(-90 至 90)
    double Roll;         //5 横滚角(-180 至 180)
    double Lattitude;     //12 纬度(-90 至 90)
    double Longitude;     //13 经度(-180 至 180)
    double Altitude;      //14 高度,单位(米)
    double Status;        //21
};

#endif 
