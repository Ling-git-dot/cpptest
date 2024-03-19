#ifndef RECEIVE_UART_H
#define RECEIVE_UART_H



#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*POSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <string.h>       /*字符串功能函数*/

#include   <signal.h>
#include    <pthread.h>
#include    <sched.h>
#include "./uart/uart.h"
#include "./gpio/gpioset.h"



//#define  USE_QUEUE    //确定是否用库函数自带的栈

#define  USE_QUEUE_THREAD  //确定是否用读写两线程

#define  USE_OWN_QUEUE    //确定是否用自己封的循环缓存的类 栈

//#define  USE_SINGLE_COM //确定是否用一个线程 收数加处理数据

#define Image_Thread_NUMBER 9
#define Moniter_Thread_NUMBER 6

extern unsigned char thread_flag_tj[Moniter_Thread_NUMBER];
extern unsigned short zg_rightcount; //在轨数据文件正确帧计数
extern unsigned short bitmap_zg_rightcount; //在轨数据文件正确帧计数  //2023.11.3
extern unsigned short  zg_wrongcount; //在轨数据文件错误帧计数
extern struct log log_data;
extern unsigned char zg_status;
extern unsigned char zg_excut;  //在轨执行
extern unsigned char enProgramModify;
//工作模式统计
extern  unsigned int bitmap_send_flag;
extern unsigned int bitmap_send_count;
extern unsigned int bitmap_send_countck;
extern unsigned int bitmap_send_flag;
extern unsigned char yc_send[2048];


//串口打印
//#define USE_PRINTF_COM0
//#define USE_PRINTF_COM_CAMERA
//#define USE_PRINTF_YC
//#define  USE_PRINTF
//#define USE_MAINTHREAD
#define  USE_ZAIGUI
#define GPIO_DOG     //看门狗的设置


//反熔丝功能都有的话是SEND_YC_YQ  SEND_YC_YQ_ONLY 都注掉，SEND_YC_TIME保留
//直通是SEND_YC_TIME注掉，那两个SEND_YC_YQ  SEND_YC_YQ_ONLY 保留
//#define SEND_YC_YQ  //xiangying

//#define SEND_YC_YQ_ONLY  //xiangying

#define SEND_YC_TIME   //dingshi

//#define   TEST_DELAY

//#define PRINT_OTHER



#define   SAVE_PIC

//#define IF_PRINTF_MONITOR
#define IF_EXPOSURE  //是否曝光
#define  HandCom   //是否模式指令断电

//#define SetIntervalExplore   //固定时间间隔的自动曝光


#ifdef  TEST_DELAY
extern  unsigned int delay_flag;
extern  unsigned int delay_flag_main;
#endif



extern long getSystime(unsigned char time[6]);

#pragma pack(1)
struct DataBuf
{
    DataBuf()
    : m_data(NULL)
    , m_len(0)
    {

    }

    DataBuf(unsigned char* data, int len)
    : m_data(data)
    , m_len(len)
    {

    }

    unsigned char*  m_data;
    int             m_len;
};
#pragma pack()


//bram

#pragma pack(1)

typedef struct
{
    unsigned int m_nEnd;
    unsigned int m_nCur;
    unsigned char *uart_data;
    unsigned char *frame_head;
    unsigned int frame_head_lenth;
    unsigned int refer_lenth;
}ReceiveData;

#pragma pack()


//点目标算法处理结果打包结果至monitor
#pragma pack(1)

#if 0
struct Piont_frame
{


    unsigned char farcamera_Alarm_staus1; //告警算法状态
    unsigned short farcamera_used_time1;  //执行时间
    unsigned char farcamera_timesamp1[6]; //时间戳
    unsigned char farcamera_distance[3];  //（改）
    short farcamera_bodyAzu;      //光学本体方位角Azimuth
    short farcamera_bodyPit;      //光学本体方位角Azimuth
    short matrix_data[9]; //矩阵 （改）
    int bodyA;//欧拉角α （改）
    int bodyB;//欧拉角β （改）
    int bodyC;//欧拉角R   （改）
    int bodyX;      //X （改）
    int bodyY;      //Y （改）
    int bodyZ;      //Z （改）


    unsigned char camera1_id; //相机1号
    unsigned char point_status1;  //算法打桩
    unsigned char Alarm_staus1; //告警算法状态
    unsigned short used_time1;  //执行时间
    unsigned char timesamp1[6]; //时间戳

    unsigned char camera1_sum; //相机1目标数
    unsigned char camera1_id0; //目标ID号
    short int bodyAzu1_0;      //光学本体方位角Azimuth
    short int bodyPit1_0;      //光学本体方位角Azimuth


    unsigned char camera1_id1; //目标ID号
    short int bodyAzu1_1;      //光学本体方位角Azimuth
    short int bodyPit1_1;      //光学本体方位角Azimuth


    unsigned char camera1_id2; //目标ID号
    short int bodyAzu1_2;      //光学本体方位角Azimuth
    short int bodyPit1_2;      //光学本体方位角Azimuth


    unsigned char camera2_id; //相机2号
    unsigned char point_status2;  //算法打桩
    unsigned char Alarm_staus2; //告警算法状态
    unsigned short used_time2;  //执行时间
    unsigned char timesamp2[6]; //时间戳
    unsigned char camera2_sum; //相机1目标数

    unsigned char camera2_id0; //目标ID号
    short int bodyAzu2_0;      //光学本体方位角Azimuth
    short int bodyPit2_0;      //光学本体方位角Azimuth


    unsigned char camera2_id1; //目标ID号
    short int bodyAzu2_1;      //光学本体方位角Azimuth
    short int bodyPit2_1;      //光学本体方位角Azimuth


    unsigned char camera2_id2; //目标ID号
    short int bodyAzu2_2;      //光学本体方位角Azimuth
    short int bodyPit2_2;      //光学本体方位角Azimuth
 };
#endif
struct Piont_frame
{
    unsigned char farcamera_Alarm_staus1; //告警算法状态
    unsigned short farcamera_used_time1;  //执行时间
    unsigned char farcamera_timesamp1[6]; //时间戳
    unsigned char farcamera_distance[3];  //（改）
    short farcamera_bodyAzu;      //光学本体方位角Azimuth
    short farcamera_bodyPit;      //光学本体方位角Azimuth
    short matrix_data[9]; //矩阵 （改）
    int bodyA;//欧拉角α （改）
    int bodyB;//欧拉角β （改）
    int bodyC;//欧拉角R   （改）
    int bodyX;      //X （改）
    int bodyY;      //Y （改）
    int bodyZ;      //Z （改）


    unsigned char camera1_id; //相机1号
    unsigned char point_status1;  //算法打桩
    unsigned char Alarm_staus1; //告警算法状态
    unsigned short used_time1;  //执行时间
    unsigned char timesamp1[6]; //时间戳

    unsigned char camera1_sum; //相机1目标数

    short int bodyAzu1_0;      //光学本体方位角Azimuth
    short int bodyPit1_0;      //光学本体方位角Azimuth
    unsigned char bright1_0; //亮度    //2023.9.11
    unsigned short bodysize1_0; //大小   //2023.9.11


    short int bodyAzu1_1;      //光学本体方位角Azimuth
    short int bodyPit1_1;      //光学本体方位角Azimuth
    unsigned char bright1_1; //亮度   //2023.9.11
    unsigned short bodysize1_1; //大小   //2023.9.11


    unsigned char camera2_id; //相机2号
    unsigned char point_status2;  //算法打桩
    unsigned char Alarm_staus2; //告警算法状态
    unsigned short used_time2;  //执行时间
    unsigned char timesamp2[6]; //时间戳
    unsigned char camera2_sum; //相机1目标数


    short int bodyAzu2_0;      //光学本体方位角Azimuth
    short int bodyPit2_0;      //光学本体方位角Azimuth
    unsigned char bright2_0; //亮度    //2023.9.11
    unsigned short bodysize2_0; //大小   //2023.9.11


   //unsigned char camera2_id1; //目标ID号
    short int bodyAzu2_1;      //光学本体方位角Azimuth
    short int bodyPit2_1;      //光学本体方位角Azimuth
    unsigned char bright2_1; //亮度   //2023.9.11
    unsigned short bodysize2_1; //大小   //2023.9.11


   unsigned char nodata[2];  //剩余的空的两字节 调整协议后 //2023.9.11
 };


#pragma pack()

#pragma pack()





#pragma pack(1)
struct Target_frame
{

    unsigned char taget_status;  //算法打桩
    unsigned char Alarm_staus; //测量算法状态
    unsigned short used_time;  //执行时间
    unsigned char timesamp[6]; //时间戳
    //unsigned int ditance_data; //距离
    unsigned char ditance_data[3];  //（改）
    short bodyAzu; //方位角  （改）
    short bodyPit; //俯仰角  （改）
    short matrix_data[9]; //矩阵 （改）
    int bodyA;//欧拉角α （改）
    int bodyB;//欧拉角β （改）
    int bodyC;//欧拉角β   （改）
    int bodyX;      //X （改）
    int bodyY;      //Y （改）
    int bodyZ;      //Z （改）
};
#pragma pack()



#pragma pack(1)
//发送除去的数据（在send dma线程发送或者单独一个线程发送）
struct Send_Monitor_frame
{

    unsigned int head; //0x1ACFFC1D 同步码
    //unsigned short framCnt; //帧计数
    //unsigned short work_mode; //工作模式

    struct Target_frame target_frame_data; //靶标算法算出的结构体内容
    struct Piont_frame point_frame_data;  //点目标算法算出的结构体内容
    unsigned char thread_flag[Image_Thread_NUMBER];
    unsigned char imagethread_status;  //  图像处理软件的线程状态  //2023.11.9 图像线程状态  喂狗

    //局部曝光增加参数
#ifdef IF_EXPOSURE
    unsigned char if_refresh_distance_flag;  //远场距离是否更新标志
    unsigned char if_refresh_near_flag;  //近场距离是否更新标志
    unsigned short x_distance; //远场x坐标值
    unsigned short y_distance; //远场y坐标值
    unsigned short distance_lenth; //区域长度
    unsigned short distance_width; //区域长度

    unsigned short x_near; //远场x坐标值
    unsigned short y_near; //远场y坐标值
    unsigned short near_lenth; //区域长度
    unsigned short near_width; //区域长度
#endif

    //unsigned char compress_qvalue;    //压缩Q值
    //unsigned char mode_change[6];
    unsigned char sum;

};
#pragma pack()


struct Image_frame
{


    //unsigned short work_mode; //工作模式
    struct Target_frame target_frame_data; //靶标算法算出的结构体内容
    struct Piont_frame point_frame_data;  //点目标算法算出的结构体内容
    unsigned char thread_flag[Image_Thread_NUMBER];

    //unsigned char compress_qvalue;    //压缩Q值

};
#pragma pack()


#pragma pack(1)
//发送给反熔丝的整包遥测数据



typedef struct
{
   unsigned short Frame_Header; //0xeb90
   unsigned char  Typeins; //指令类型 0xf0
   unsigned short EPDU_Lenth;  //EPDU长
   unsigned short  Mutiple_lenth;

   unsigned short Packet_Identity; // 包识别
   unsigned short  Packet_Control;  //包序列控制
   unsigned short Packet_Lenth;  // 包长

   //shenzong遥测
   //unsigned char fj_data[12];  //复接软件的遥测量
   unsigned int fj_data[3];


   //monitor软件遥测
   unsigned char ins_count; // 指令计数
   unsigned char  wrong_ins_count;//错误指令计数
   unsigned char  framesz_count; //上注包计数
   unsigned char  wrong_framesz_count;//上注包错误计数
   unsigned char  version; //版本
   //image软件图像遥测
   unsigned short work_mode; //工作模式

   struct Target_frame target_frame_data; //靶标算法算出的结构体内容
   struct Piont_frame point_frame_data;  //点目标算法算出的结构体内容
   //unsigned char compress_qvalue;    //压缩Q值

   unsigned char thread_flag[8];  //其他自己看的状态  线程状态
   unsigned char status_zt; //数传开关状态，在轨编程状态，工作模式
   unsigned short zg_rightcount; //在轨空间文件正确帧计数
   unsigned short zg_wrongcount; //在轨空间文件错误帧计数
   unsigned char software_resetcount; //软复位帧计数
   unsigned char resetfactory_count;//恢复出厂设置帧计数
   unsigned short bitmap_lenthall;  //bitmap总长度
   //unsigned char free_data[2];//未使用的2字节 //2023.11.3
   unsigned short bitmap_zg_rightcount;  //2023.11.3



   //6路相机的状态（baobin）
   unsigned  char camera_data[24];//baobin 18字节

   unsigned  char camerayasuo[8]; //h265压缩

   unsigned char data_out[2]; //lifan


   unsigned char sum;
} tmPackage;

#pragma pack()





//3559编码模块计数
#pragma pack(1)
typedef struct
{
    unsigned int m_framehead;  //帧头0x1ACFFC1E
    unsigned short frame_count;//帧计数
    unsigned char encode_ins_count;//编码指令计数
    unsigned short wrong_ins_cout; //错误指令计数
    char version_number[3]; //版本号
    unsigned int receive_correctimage_count;//接收正确图像计数
    unsigned int receive_wrongimage_count;//接收错误图像计数
    unsigned int encode_correct_cout; //正确编码计数
    unsigned int frame_tail;//帧尾  0x47474747
    unsigned short  sum; //最后一个字节填0xaa
}EncodeModule;
#pragma pack()
//图像复接模块
#pragma pack(1)
typedef struct
{
    unsigned int m_framehead;  //帧头0x1ACFFC1F
    unsigned short frame_count;//帧计数
    unsigned char encode_ins_count;//编码指令计数
    unsigned short wrong_ins_cout; //错误指令计数
    char version_number[3]; //版本号
    unsigned int receive_correctimage_count;//接收正确图像计数
    unsigned int receive_wrongimage_count;//接收错误图像计数
    unsigned int encode_correct_cout; //正确编码计数
    unsigned int frame_tail;//帧尾  0x47474747
    unsigned short  sum; //最后一个字节填0xaa
}MutilConnect;

#pragma pack()

extern int fd[7];
extern char port1[];
extern char port2[];
extern char port3[];
extern char port4[];
extern char port5[];
extern char port6[];
extern char port7[];
extern void *uart_recv_msg(void *arg);
extern void *uart_recv_msg_write(void *arg);
extern void *uart_recv_msg_read(void *arg);
extern void *uart_recv_msg_write_camera(void *arg);
extern void *uart_recv_msg_read_camera(void *arg);
extern void InitUart();
extern void InitUartMemory();
extern void DestroyUartMemory();
extern unsigned char checkSum(unsigned char cmd[], int len, unsigned char checkSum);
extern unsigned char checkSum1(unsigned char cmd[], int len, unsigned char checkSum);
extern void TelctlProcess(unsigned char *data);
extern void telctl_feedback(int fd_send,char flag_send);
extern unsigned char cmputCheckSum(unsigned char data[],int len);
extern unsigned int  InitTmData(tmPackage *tmdatatemp);
extern unsigned int SendTmData(tmPackage *tmdatatemp);
extern void ZhData(tmPackage *data_in,tmPackage *data_out);
extern tmPackage  tmdata;
extern tmPackage  tmdata_out; //高低字节转换
extern pthread_mutex_t mutex;
extern const char localIpAddr[];
extern int local_port;
extern const char dstIpAddr[];
extern int dst_port;
extern unsigned char mode_if_change[6];
extern void InitBramMonitor();
extern void ModifyFjyk();
extern void SetWorkmode(unsigned short &data_in);
extern void SetCameraMode();
extern unsigned int camera_setmode;
//局部曝光增加参数
extern unsigned char  data_exposure_flag[6];  //6路相机是否局部曝光
extern unsigned char  data_exposure[6][12];  //6行16列  数据

//增加时钟配置的函数
extern void InitBramClock();
extern void SetClockFre(unsigned int flag);

//发送bitmap的yc
extern int SendBitMapYcCount();
int SendBitMapYc(tmPackage *tmdata,unsigned char *data);

//2023.12.21
extern void InitGpio();
extern int InitGpioout(unsigned int flag,unsigned int channel);

#ifdef GPIO_DOG

extern unsigned char if_send_yc_image;
extern unsigned char if_send_yc_monitor;
#endif


void setcalibrationTime(unsigned char gncTime[]);

//宏定义
#define FALSE  -1
#define TRUE   0




#endif // REICEIVE_UART_H
