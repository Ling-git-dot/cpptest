#include "receive_uart.h"
#include "receive_udp.h"
#include"./bram/bram.h"
#include "MemPool.h"
#include "NonBlockQueue.h"
//#include <queue>   //利用pop和push开两个线程一个用于收网络，一个用于处理网络
#include"online_program.h"
#include "receive_udp.h"
#include <sys/time.h>
#include<sys/wait.h>





using namespace std;


//char port1[]="/dev/ttyPS19";
//char port2[]="/dev/ttyPS23";
//char port3[]="/dev/ttyPS3";
//char port4[]="/dev/ttyPS4";
//char port5[]="/dev/ttyPS5";
//char port6[]="/dev/ttyPS6";
//char port7[]="/dev/ttyPS7";

//char port1[]="/dev/pts/19";
//char port2[]="/dev/pts/23";
//char port3[]="/dev/pts/19";
//char port4[]="/dev/pts/19";
//char port5[]="/dev/pts/19";
//char port6[]="/dev/pts/19";
//char port7[]="/dev/pts/19";
#define PORT_NUMBER 7
#define Port_Lenth  2048
#define Refer_Lenth_Camera 5
#define Port_Lenth_Max 4*2048*2048
//const char *port[]={"/dev/pts/21","/dev/pts/23","/dev/pts/1","/dev/pts/2","/dev/pts/3","/dev/pts/19","/dev/pts/19"};
const char *port[]={"/dev/ttyUL1","/dev/ttyUL2","/dev/ttyUL3","/dev/ttyUL4","/dev/ttyUL5","/dev/ttyUL6","/dev/ttyUL7"};
//const char *port[]={"/dev/pts/21","/dev/pts/35","/dev/pts/41","/dev/pts/42","/dev/pts/43","/dev/pts/44","/dev/pts/45"};

//const char *port[]={"/dev/pts/22","/dev/pts/311","/dev/pts/341","/dev/pts/221","/dev/pts/371","/dev/pts/401","/dev/pts/431"};
int fd[7];
//unsigned char frame_head_encode[]={0x1A,0xCF,0xFC,0x1E};  //3559图像编码软件
//unsigned char frame_head_mutiple[]={0x1A,0xCF,0xFC,0x1D};  //图像复接软件
unsigned char frame_head_camera[]={0xEB,0x90};  //图像相机软件
unsigned char frame_head_telctl[]={0xEB,0x90};  //遥控指令帧头
//遥控注入指令表
unsigned char telctl_list[55][4]={
    {0x01,0x01,0x01,0x01},
    {0x02,0x02,0x02,0x02},
    {0x03,0x03,0x03,0x03},
    {0x04,0x04,0x04,0x04},
    {0x05,0x05,0x05,0x05},
    {0x06,0x06,0x06,0x06},
    {0x07,0x07,0x07,0x07},
    {0x08,0x08,0x08,0x08},
    {0x09,0x09,0x09,0x09},
    {0x0a,0x0a,0x0a,0x0a},
    {0x0b,0x0b,0x0b,0x0b},
    {0x0c,0x0c,0x0c,0x0c},
    {0x0d,0x0d,0x0d,0x0d},
    {0x0e,0x0e,0x0e,0x0e},
    {0x0f,0x0f,0x0f,0x0f},
    {0x10,0x10,0x10,0x10},
    {0x11,0x11,0x11,0x11},
    {0x12,0x12,0x12,0x12},
    {0x13,0x13,0x13,0x13},
    {0x14,0x14,0x14,0x14},
    {0x15,0x15,0x15,0x15},
    {0x16,0x16,0x16,0x16},
    {0x17,0x17,0x17,0x17},
    {0x18,0x18,0x18,0x18},
    {0x19,0x19,0x19,0x19},
    {0x20,0x20,0x20,0x20},
    {0x21,0x21,0x21,0x21},
    {0x22,0x22,0x22,0x22},
    {0x23,0x23,0x23,0x23},
    {0x24,0x24,0x24,0x24},
    {0x25,0x25,0x25,0x25},
    {0x26,0x26,0x26,0x26},
    {0x27,0x27,0x27,0x27},
    {0x28,0x28,0x28,0x28},
    {0x29,0x29,0x29,0x29},
    {0x2a,0x2a,0x2a,0x2a},
    {0x2b,0x2b,0x2b,0x2b},
    {0x2c,0x2c,0x2c,0x2c},
    {0x2d,0x2d,0x2d,0x2d},
    {0x2e,0x2e,0x2e,0x2e},
    {0x2f,0x2f,0x2f,0x2f},
    {0x30,0x30,0x30,0x30},
    {0x31,0x31,0x31,0x31},
    {0x32,0x32,0x32,0x32},
    {0x33,0x33,0x33,0x33},
    {0x34,0x34,0x34,0x34},
    {0x35,0x35,0x35,0x35},
    {0x36,0x36,0x36,0x36},
    {0x37,0x37,0x37,0x37},
    {0x38,0x38,0x38,0x38},
    {0x39,0x39,0x39,0x39},
    {0x3a,0x3a,0x3a,0x3a},
    {0x3b,0x3b,0x3b,0x3b},
    {0x3c,0x3c,0x3c,0x3c},
    {0x3d,0x3d,0x3d,0x3d}
};


//接收缓存
//unsigned  char *data_uart_in[PORT_NUMBER];
ReceiveData data_uart_process[PORT_NUMBER];

//bram的格式
bramInfo m_bramInfo;
unsigned int baseAddr=0xa000000001;
int memsize=1024;

//遥测变量
tmPackage  tmdata;
tmPackage  tmdata_out; //高低字节转换


//更新遥测时候的锁
pthread_mutex_t mutex;

//应用库函数queue
#ifdef  USE_QUEUE
queue<char>q_uart;
queue<char>q_cam1;
queue<char>q_cam2;
queue<char>q_cam3;
queue<char>q_cam4;
queue<char>q_cam5;
queue<char>q_cam6;
queue<char>q_encodemodule;
#endif

//是否进入只响应保护指令阶段
unsigned char enProgramModify = 0xff;
int flag_power_off = 0;
//应用自己封装的类
#ifdef  USE_OWN_QUEUE
#define Max_Number 400
//#define UART_LENTH 1024
//#define UART_LENTH_recv 512

#define UART_LENTH 2048
#define UART_LENTH_recv 1024
zbl::MemPool<unsigned char,UART_LENTH>     g_mem_pool_uart[7]; //开辟1024个char*20个缓存
zbl::NonBlockQueue<DataBuf,Max_Number>             g_uart_que[7];  //20 block
#endif


//发送给网络的指令封装
#define send_net_number 69
unsigned char send_net[send_net_number];

//串口发送数据

#define send_uart_number_temp 6

#ifndef IF_EXPOSURE
#define send_uart_number 9
unsigned char send_uart_camera[send_uart_number];
#endif


#ifdef IF_EXPOSURE
#define send_uart_number 12
unsigned char send_uart_camera[send_uart_number];
#endif
unsigned char cmd_send[send_uart_number_temp];




const char localIpAddr[]={"127.0.0.2"};
//const char localIpAddr[]={"192.168.1.90"};
int local_port=6002;

const char dstIpAddr[]={"127.0.0.3"};
//const char dstIpAddr[]={"192.168.1.10"};
int dst_port=6003;

//获取指令
unsigned char mode_if_free[6]={0,0,0,0,0,0};
//unsigned char mode_if_free[6]={1,1,1,1,1,1};
//unsigned char mode_if_change[6]={4,5,3,2,1,0};
//unsigned char mode_if_change[6]={3,4,0,1,2,5};
//unsigned char mode_if_change[6]={2,4,0,1,3,5};  //5.11以前的版本

//unsigned char mode_if_change[6]={4,1,3,5,2,0};  //5.11 IDS更改后的 广域c 远场 广域a  广域b 广域d  近场
unsigned char mode_if_change[6]={1,0,3,5,4,2};


unsigned char mode_lx=0; //是都是轮循环模式

//bram的操作
//2023.3.8
bramInfo pBramInfo[3];
//和pl通信的共享缓存
#define SHARE_YC 0x80150000
#define  BRAM_LENTH  512
#define  SHARE_CLOCK 0x80000000
#define  BRAM_LENTH_CLOCK 1024

//5.11以前的版本
/*
unsigned char mode_list[7][6]={{0,1,2,3,4,5},  //0D广域A&B告警、广域C&D&远视场&近视场监视
                               {0,2,1,3,4,5},   //0E广域A&C告警、广域B&D&远视场&近视场监视
                               {0,3,1,2,4,5},   //0F广域A&D告警、广域B&C&远视场&近视场监视
                               {1,2,0,3,4,5},   //10广域B&C告警、广域A&D&远视场&近视场监视
                               {1,3,0,2,4,5},   //11广域B&D告警、广域A&C&远视场&近视场监视
                               {2,3,0,1,4,5},   //12广域C&D告警、广域A&B&远视场&近视场监视
                               {4,5,2,3,0,1},   //13远近视场协同测量、广域A&B&C&D监视
                              };
                              */
//unsigned char mode_if_changesend[6]={2,4,0,1,3,5};  //IDS更改后的 广域c 远场 广域a  广域b 广域d  近场
unsigned char mode_if_changesend[6]={4,5,0,1,2,3};

//2023.12.28删除
//unsigned char mode_listsend[15][6]={{0,1,2,3,4,5},  //0D广域A&B告警、广域C&D&远视场&近视场监视
//                               {0,2,1,3,4,5},   //0E广域A&C告警、广域B&D&远视场&近视场监视
//                               {0,3,1,2,4,5},   //0F广域A&D告警、广域B&C&远视场&近视场监视
//                               {1,2,0,3,4,5},   //10广域B&C告警、广域A&D&远视场&近视场监视
//                               {1,3,0,2,4,5},   //11广域B&D告警、广域A&C&远视场&近视场监视
//                               {2,3,0,1,4,5},   //12广域C&D告警、广域A&B&远视场&近视场监视
//                               {4,5,0,1,2,3},   //13远近视场协同测量、广域A&B&C&D监视
//                               {0,4,1,2,3,5},  //广域A告警&远场测量，广域B&C&D&近视场监视  //2023.9.11
//                               {1,4,0,2,3,5},  //广域B告警&远场测量，广域A&C&D&近视场监视  //2023.9.11
//                               {2,4,0,1,3,5}, //广域C告警&远场测量，广域A&B&D&近视场监视  //2023.9.11
//                               {3,4,0,1,2,5}, //广域D告警&远场测量，广域A&B&D&近视场监视  //2023.9.11
//                               {0,5,1,2,3,4},  //广域A告警&近场测量，广域B&C&D&远视场监视  //2023.9.11
//                               {1,5,0,2,3,4},  //广域B告警&近场测量，广域A&C&D&远视场监视  //2023.9.11
//                               {2,5,0,1,3,4}, //广域C告警&近场测量，广域A&B&D&远视场监视  //2023.9.11
//                               {3,5,0,1,2,4}, //广域D告警&近场测量，广域A&B&D&远视场监视  //2023.9.11
//                              };


////5.15end根据IDS更改的版本
//unsigned char mode_list[15][6]={{3,5,4,2,1,0},  //0D广域A&B告警、广域C&D&远视场&近视场监视
//                               {3,4,5,2,1,0},   //0E广域A&C告警、广域B&D&远视场&近视场监视
//                               {3,2,5,4,1,0},   //0F广域A&D告警、广域B&C&远视场&近视场监视
//                               {5,4,3,2,1,0},   //10广域B&C告警、广域A&D&远视场&近视场监视
//                               {5,2,3,4,1,0},   //11广域B&D告警、广域A&C&远视场&近视场监视
//                               {4,2,3,5,1,0},   //12广域C&D告警、广域A&B&远视场&近视场监视
//                               {1,0,3,5,4,2},   //13远近视场协同测量、广域A&B&C&D监视
//                               {3,1,5,4,2,0},  //广域A告警&远场测量，广域B&C&D&近视场监视  //2023.9.11
//                               {5,1,3,4,2,0},  //广域B告警&远场测量，广域A&C&D&近视场监视  //2023.9.11
//                               {4,1,3,5,2,0}, //广域C告警&远场测量，广域A&B&D&近视场监视  //2023.9.11
//                               {2,1,3,5,4,0}, //广域D告警&远场测量，广域A&B&C&近视场监视  //2023.9.11
//                               {3,0,5,4,2,1},  //广域A告警&近场测量，广域B&C&D&远视场监视  //2023.9.11
//                               {5,0,3,4,2,1},  //广域B告警&近场测量，广域A&C&D&远视场监视  //2023.9.11
//                               {4,0,3,5,2,1}, //广域C告警&近场测量，广域A&B&D&远视场监视  //2023.9.11
//                               {2,0,3,5,4,1}, //广域D告警&近场测量，广域A&B&C&远视场监视  //2023.9.11
//                              };



unsigned char mode_listsend[15][6]={{0,1,2,3,4,5},  //0D广域A&B告警、广域C&D&远视场&近视场监视
                               {0,2,1,3,4,5},   //0E广域A&C告警、广域B&D&远视场&近视场监视
                               {0,3,1,2,4,5},   //0F广域A&D告警、广域B&C&远视场&近视场监视
                               {1,2,0,3,4,5},   //10广域B&C告警、广域A&D&远视场&近视场监视
                               {1,3,0,2,4,5},   //11广域B&D告警、广域A&C&远视场&近视场监视
                               {2,3,0,1,4,5},   //12广域C&D告警、广域A&B&远视场&近视场监视
                               {4,5,0,1,2,3},   //13远近视场协同测量、广域A&B&C&D监视
                               {4,0,1,2,3,5},  //远场测量&广域A告警，广域B&C&D&近视场监视  //2023.9.11
                               {4,1,0,2,3,5},  //远场测量&广域B告警，广域A&C&D&近视场监视  //2023.9.11
                               {4,2,0,1,3,5}, //远场测量&广域C告警，广域A&B&D&近视场监视  //2023.9.11
                               {4,3,0,1,2,5}, //远场测量&广域D告警，广域A&B&D&近视场监视  //2023.9.11
                               {0,5,1,2,3,4},  //广域A告警&近场测量，广域B&C&D&远视场监视  //2023.9.11
                               {1,5,0,2,3,4},  //广域B告警&近场测量，广域A&C&D&远视场监视  //2023.9.11
                               {2,5,0,1,3,4}, //广域C告警&近场测量，广域A&B&D&远视场监视  //2023.9.11
                               {3,5,0,1,2,4}, //广域D告警&近场测量，广域A&B&D&远视场监视  //2023.9.11
                              };


unsigned char mode_list[15][6]={{3,5,4,2,1,0},  //0D广域A&B告警、广域C&D&远视场&近视场监视
                               {3,4,5,2,1,0},   //0E广域A&C告警、广域B&D&远视场&近视场监视
                               {3,2,5,4,1,0},   //0F广域A&D告警、广域B&C&远视场&近视场监视
                               {5,4,3,2,1,0},   //10广域B&C告警、广域A&D&远视场&近视场监视
                               {5,2,3,4,1,0},   //11广域B&D告警、广域A&C&远视场&近视场监视
                               {4,2,3,5,1,0},   //12广域C&D告警、广域A&B&远视场&近视场监视
                               {1,0,3,5,4,2},   //13远近视场协同测量、广域A&B&C&D监视
                               {1,3,5,4,2,0},  //远场测量&广域A告警，广域B&C&D&近视场监视  //2023.9.11
                               {1,5,3,4,2,0},  //远场测量&广域B告警，广域A&C&D&近视场监视  //2023.9.11
                               {1,4,3,5,2,0}, //远场测量&广域C告警，广域A&B&D&近视场监视  //2023.9.11
                               {1,2,3,5,4,0}, //远场测量&广域D告警，广域A&B&C&近视场监视  //2023.9.11
                               {3,0,5,4,2,1},  //广域A告警&近场测量，广域B&C&D&远视场监视  //2023.9.11
                               {5,0,3,4,2,1},  //广域B告警&近场测量，广域A&C&D&远视场监视  //2023.9.11
                               {4,0,3,5,2,1}, //广域C告警&近场测量，广域A&B&D&远视场监视  //2023.9.11
                               {2,0,3,5,4,1}, //广域D告警&近场测量，广域A&B&C&远视场监视  //2023.9.11
                              };

/*
unsigned char mode_list[7][6]={{0,1,2,3,4,5},  //0D广域A&B告警、广域C&D&远视场&近视场监视
                               {0,2,4,5,1,3},   //0E广域A&C告警、广域B&D&远视场&近视场监视
                               {0,3,1,2,4,5},   //0F广域A&D告警、广域B&C&远视场&近视场监视
                               {1,2,0,3,4,5},   //10广域B&C告警、广域A&D&远视场&近视场监视
                               {1,3,0,2,4,5},   //11广域B&D告警、广域A&C&远视场&近视场监视
                               {2,3,0,1,4,5},   //12广域C&D告警、广域A&B&远视场&近视场监视
                               {4,5,2,3,0,1},   //13远近视场协同测量、广域A&B&C&D监视
                              };
                              */







//发送串口0 反熔丝要用到的数
#if 0
 unsigned char data_in_com0[UART_LENTH];
 unsigned char data_in_com0_lenth=0;

 unsigned char data_in_com1[UART_LENTH];
 unsigned char data_in_com1_lenth=0;
 unsigned char data_in_com2[UART_LENTH];
 unsigned char data_in_com2_lenth=0;
 unsigned char data_in_com3[UART_LENTH];
 unsigned char data_in_com3_lenth=0;
 unsigned char data_in_com4[UART_LENTH];
 unsigned char data_in_com4_lenth=0;
 unsigned char data_in_com5[UART_LENTH];
 unsigned char data_in_com6_lenth=0;
 unsigned char data_in_com6[UART_LENTH];
 unsigned char data_in_com6_lenth=0;
#endif



unsigned char work_mode_tj=0;  //默认是在工作模式 0 工作 1 测试 2 自检
unsigned char sc_on_off=1;  //数传开关状态 0 关 1开
unsigned char zg_status=0; //在轨编程状态  0 未在轨编程状态  1 在轨编程模式
unsigned char zg_excut=0; // 0未在轨编程实施  1在轨实施
unsigned char clock_status=0; //0 20M 1 10M

//unsigned short zg_rightcount; //在轨数据文件正确帧计数
//unsigned short  zg_wrongcount; //在轨数据文件错误帧计数
//unsigned char   softresetcount; //软复位帧计数
//unsigned char resetfactorycount; //恢复出厂设置帧计数
struct log log_data;

unsigned int camera_setmode=0;  //设置相机模式

//统计线程
unsigned char thread_flag_tj[Moniter_Thread_NUMBER];


//用于发送遥测的数据
unsigned char yc_send[2048];
unsigned char yc_bitmap_data[2048];
unsigned char yc_bitmap_data_file[2048];
unsigned int yc_bitmap_data_file_lenth;
unsigned int bitmap_setcount=0;
unsigned short bitmap_epfucount=0x8000;  //还要代表最后一包10
int bitmap_lenth;
unsigned int bitmap_zj=0;
unsigned int bitmap_ys=0;
unsigned int bitmap_mode=0;

unsigned short EPDU_LenthAddBitmap;  //EPDU长
unsigned short  Mutiple_lenthAddBitmap;

unsigned int sum_bitmap=0;
unsigned char sum_epdu=0;
#define BTIMAP_BLOCK 238

unsigned int bitmap_send_flag=0;
unsigned int bitmap_send_count=0;
unsigned int bitmap_send_countck=0;
unsigned char bitmap_success=0;
//unsigned int resetFactory_flag=0;

#ifdef GPIO_DOG
unsigned char if_send_yc_image=0;
unsigned char if_send_yc_monitor=0;
#endif

//2023.12.28
unsigned int roll_flag_mode=0;


//局部曝光增加参数
unsigned char  data_exposure_flag[6];  //6路相机是否局部曝光
unsigned char  data_exposure[6][12];  //6行16列  数据

//2023.4.28  软复位、硬复位、恢复出厂设置
/*****************************************************
 * @brief 恢复出厂设置
 * 具体操作内容为替换/usr/lib中的动态库，/usr/bin中的可执行程序
 * 直接运行的命令行
 *****************************************************/

//void resetFactory()
//{
//    printf("reset to Factory setting... \n");
//    char reset_process_factory[] = "cp -r -f /spiflash/backup/process/* /spiflash/";
//    FILE* fp_1 = popen(reset_process_factory, "r");
//    //char reset_library_factory[] = "cp -r -f /usr/backup/library/* /usr/lib/ ";
//    //FILE* fp_2 = popen(reset_library_factory, "r");
//    pclose(fp_1);
//   // pclose(fp_2);
//    printf("reset to Factory setting finished! \n");
//}




unsigned int GetNumber(unsigned char *data_in,unsigned int number)
{
    unsigned int i;
    for(i=0;i<6;i++)
    {
        if(mode_if_changesend[i]==number)
         {
            break;
          }
    }

    if(i<2)
     {
        return 0;
    }
    else
    {
        return 1;
    }

}

//2023.12.21
void InitGpio()
{
    unsigned int i;
    for(i=4;i<10;i++)
    {
        initGpio(i);
        setGpioDirection(i,(char*)("out"));
        //setGpioValue()

    }

}

//2023.12.21
int InitGpioout(unsigned int flag,unsigned int channel)
{
    //unsigned int i;
    bool  flag_true;
    /*
    for(i=0;i<32;i++)
    {
        initGpio(32+i);
        setGpioDirection(32+i,(char*)("in"));
    }
    */
   // for(i=4;i<10;i++)
    //{
        //initGpio(i);
        //setGpioDirection(i,(char*)("out"));
        //setGpioValue()
        //gpio_level_set(i,flag);
   // }

    flag_true=gpio_level_set(channel,flag);

    return flag_true;



}

int execresetFactory(char *filedata,  char *filedatamd5)
{
    // 构建shell命令
    char command[256];
    char md5sum[33];
    char stored_md5sum[33];
    sprintf(command, "md5sum %s | awk '{print $1}'", filedata);

    // 执行shell命令并获取输出结果
    FILE *fp = popen(command, "r");
    if (fp == NULL) {
        #ifdef IF_PRINTF_MONITOR
        printf("Failed to execute command\n");
        #endif
        return -1;
    }
    fgets(md5sum, sizeof(md5sum), fp);

    int status = pclose(fp);
    if (status == -1) {
       #ifdef IF_PRINTF_MONITOR
       printf("Failed to close command\n");
       #endif
       return -1;
    }

    // 检查文件是否存在
    if (access(filedata, F_OK) == -1) {
        #ifdef IF_PRINTF_MONITOR
        printf("File does not exist\n");
        #endif
        return -1;
    }

    // 去除换行符
    md5sum[strcspn(md5sum, "\n")] = '\0';


    // 读取1md5.txt中的校验和
    FILE *md5file= fopen(filedatamd5, "r");
    if (md5file == NULL)
    {
       #ifdef IF_PRINTF_MONITOR
       printf("Failed to open md5 file\n");
       #endif
       return -1;
    }
    fgets(stored_md5sum, sizeof(stored_md5sum), md5file);
    fclose(md5file);
    // 去除换行符
    stored_md5sum[strcspn(stored_md5sum, "\n")] = '\0';

    // 比较md5校验值
    if (strcmp(md5sum, stored_md5sum) == 0) {
        #ifdef IF_PRINTF_MONITOR
        printf("MD5 checksums match\n");
        #endif
        return 0;
    } else {
        #ifdef IF_PRINTF_MONITOR
        printf("MD5 checksums do not match\n");
        #endif
        return -1;
    }
}


int resetFactory1()
{
    int ret;
    char result[2048] = {0};
    #ifdef IF_PRINTF_MONITOR
    printf("reset to Factory setting... \n");
    #endif
    unsigned int count=0;




    ret=execresetFactory((char*)"/spiflash/backup1/image_process", (char*)"/spiflash/backup1/imageprocess.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /spiflash/backup1/image_process /usr/bin  && cp /spiflash/backup1/image_process /spiflash/usr_process_1  && cp /spiflash/backup1/image_process /spiflash/usr_process_2 &&cp /spiflash/backup1/image_process /spiflash/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /spiflash/backup1/imageprocess.txt.md5 /usr/bin  && cp /spiflash/backup1/imageprocess.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup1/imageprocess.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup1/imageprocess.txt.md5 /spiflash/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy image_process success\n");
        #endif

    }



    ret=execresetFactory((char*)"/spiflash/backup1/monitor", (char*)"/spiflash/backup1/monitor.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /spiflash/backup1/monitor /usr/bin  && cp /spiflash/backup1/monitor /spiflash/usr_process_1  && cp /spiflash/backup1/monitor /spiflash/usr_process_2 &&cp /spiflash/backup1/monitor /spiflash/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /spiflash/backup1/monitor.txt.md5 /usr/bin  && cp /spiflash/backup1/monitor.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup1/monitor.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup1/monitor.txt.md5 /spiflash/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy monitor success\n");
        #endif

    }


    ret=execresetFactory((char*)"/spiflash/backup1/poly4_S_HYG_full_smag55_dangle-35.txt", (char*)"/spiflash/backup1/poly4.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /spiflash/backup1/poly4_S_HYG_full_smag55_dangle-35.txt  /usr/bin  && cp /spiflash/backup1/poly4_S_HYG_full_smag55_dangle-35.txt /spiflash/usr_process_1  && cp /spiflash/backup1/poly4_S_HYG_full_smag55_dangle-35.txt /spiflash/usr_process_2 &&cp /spiflash/backup1/poly4_S_HYG_full_smag55_dangle-35.txt /spiflash/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /spiflash/backup1/poly4.txt.md5 /usr/bin  && cp /spiflash/backup1/poly4.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup1/poly4.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup1/poly4.txt.md5 /spiflash/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy poly4 success\n");
        #endif

    }



    ret=execresetFactory((char*)"/spiflash/backup1/Shanghai-804-Satallite-20210511-ID.prj", (char*)"/spiflash/backup1/Shanghai.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /spiflash/backup1/Shanghai-804-Satallite-20210511-ID.prj  /usr/bin  && cp /spiflash/backup1/Shanghai-804-Satallite-20210511-ID.prj /spiflash/usr_process_1  && cp /spiflash/backup1/Shanghai-804-Satallite-20210511-ID.prj /spiflash/usr_process_2 &&cp /spiflash/backup1/Shanghai-804-Satallite-20210511-ID.prj /spiflash/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /spiflash/backup1/Shanghai.txt.md5 /usr/bin  && cp /spiflash/backup1/Shanghai.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup1/Shanghai.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup1/Shanghai.txt.md5 /spiflash/usr_process_3",result);
        if(ret==0)
        {
           count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy Shanghai-804-Satallite-20210511-ID.prj success\n");
        #endif

    }


    if(count==8)
    {
        ret=0;
    }
    else
    {
        ret=-1;
    }
    #ifdef IF_PRINTF_MONITOR
    printf("ret  count %d\n",count);
    #endif
    return ret;






}
//恢复出厂设置
//先校验 后复制  2023.12.15
int resetFactory()
{
    int ret;
    char result[2048] = {0};
    #ifdef IF_PRINTF_MONITOR
    printf("reset to Factory setting... \n");
    #endif
    unsigned int count=0;




   ret=execresetFactory((char*)"/spiflash/backup/image_process", (char*)"/spiflash/backup/imageprocess.txt.md5");
   if(ret==0)
    {
       ret=ExecuteCMD((char*)"cp /spiflash/backup/image_process /usr/bin  && cp /spiflash/backup/image_process /spiflash/usr_process_1  && cp /spiflash/backup/image_process /spiflash/usr_process_2 &&cp /spiflash/backup/image_process /spiflash/usr_process_3",result);
       if(ret==0)
       {
           count++;
       }

       ret=ExecuteCMD((char*)"cp /spiflash/backup/imageprocess.txt.md5 /usr/bin  && cp /spiflash/backup/imageprocess.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup/imageprocess.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup/imageprocess.txt.md5 /spiflash/usr_process_3",result);
       if(ret==0)
       {
           count++;
       }

       #ifdef IF_PRINTF_MONITOR
       printf("copy image_process success\n");
       #endif

   }



   ret=execresetFactory((char*)"/spiflash/backup/monitor", (char*)"/spiflash/backup/monitor.txt.md5");
   if(ret==0)
    {
       ret=ExecuteCMD((char*)"cp /spiflash/backup/monitor /usr/bin  && cp /spiflash/backup/monitor /spiflash/usr_process_1  && cp /spiflash/backup/monitor /spiflash/usr_process_2 &&cp /spiflash/backup/monitor /spiflash/usr_process_3",result);
       if(ret==0)
       {
           count++;
       }

       ret=ExecuteCMD((char*)"cp /spiflash/backup/monitor.txt.md5 /usr/bin  && cp /spiflash/backup/monitor.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup/monitor.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup/monitor.txt.md5 /spiflash/usr_process_3",result);
       if(ret==0)
       {
           count++;
       }

       #ifdef IF_PRINTF_MONITOR
       printf("copy monitor success\n");
       #endif

   }


   ret=execresetFactory( (char*)"/spiflash/backup/poly4_S_HYG_full_smag55_dangle-35.txt", (char*)"/spiflash/backup/poly4.txt.md5");
   if(ret==0)
    {
       ret=ExecuteCMD((char*)"cp /spiflash/backup/poly4_S_HYG_full_smag55_dangle-35.txt  /usr/bin  && cp /spiflash/backup/poly4_S_HYG_full_smag55_dangle-35.txt /spiflash/usr_process_1  && cp /spiflash/backup/poly4_S_HYG_full_smag55_dangle-35.txt /spiflash/usr_process_2 &&cp /spiflash/backup/poly4_S_HYG_full_smag55_dangle-35.txt /spiflash/usr_process_3",result);
       if(ret==0)
       {
           count++;
       }

       ret=ExecuteCMD((char*)"cp /spiflash/backup/poly4.txt.md5 /usr/bin  && cp /spiflash/backup/poly4.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup/poly4.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup/poly4.txt.md5 /spiflash/usr_process_3",result);
       if(ret==0)
       {
           count++;
       }

       #ifdef IF_PRINTF_MONITOR
       printf("copy poly4 success\n");
       #endif

   }



   ret=execresetFactory((char*)"/spiflash/backup/Shanghai-804-Satallite-20210511-ID.prj", (char*)"/spiflash/backup/Shanghai.txt.md5");
   if(ret==0)
    {
       ret=ExecuteCMD((char*)"cp /spiflash/backup/Shanghai-804-Satallite-20210511-ID.prj  /usr/bin  && cp /spiflash/backup/Shanghai-804-Satallite-20210511-ID.prj /spiflash/usr_process_1  && cp /spiflash/backup/Shanghai-804-Satallite-20210511-ID.prj /spiflash/usr_process_2 &&cp /spiflash/backup/Shanghai-804-Satallite-20210511-ID.prj /spiflash/usr_process_3",result);
       if(ret==0)
       {
           count++;
       }

       ret=ExecuteCMD((char*)"cp /spiflash/backup/Shanghai.txt.md5 /usr/bin  && cp /spiflash/backup/Shanghai.txt.md5 /spiflash/usr_process_1  && cp /spiflash/backup/Shanghai.txt.md5 /spiflash/usr_process_2 &&cp /spiflash/backup/Shanghai.txt.md5 /spiflash/usr_process_3",result);
       if(ret==0)
       {
          count++;
       }

       #ifdef IF_PRINTF_MONITOR
       printf("copy Shanghai-804-Satallite-20210511-ID.prj success\n");
       #endif

   }


   if(count==8)
   {
       ret=0;
   }
   else
   {
       ret=-1;
   }
   #ifdef IF_PRINTF_MONITOR
   printf("ret  count %d\n",count);
   #endif
   return ret;




#if 0
    ret=ExecuteCMD("cp -r -f /spiflash/backup/process/* /spiflash/",result);
    //ret=ExecuteCMD("cp -r -f /spiflash/backup/process/image_process /spiflash/",result);


    if(ret==0)
     {
        #ifdef IF_PRINTF_MONITOR
         printf("reset to Factory setting finished! \n");
        #endif
    }
    else
     {
        #ifdef IF_PRINTF_MONITOR
        printf("reset to Factory failed! \n");
        #endif
    }


    return ret;
#endif

}


int resetFactorytest()
{
    int ret;
    char result[2048] = {0};
    #ifdef IF_PRINTF_MONITOR
    printf("reset to Factory setting... \n");
    #endif
    unsigned int count=0;



#if 0
    ret=ExecuteCMD("cp -r -f /mnt/spiflashtxt/backup/process/* /mnt/spiflashtxt/",result);
    //ret=ExecuteCMD("cp -r -f /mnt/spiflashtxt/backup/process/image_process /mnt/spiflashtxt/",result);


    if(ret==0)
     {
        #ifdef IF_PRINTF_MONITOR
         printf("reset to Factory setting finished! \n");
        #endif
    }
    else
     {
        #ifdef IF_PRINTF_MONITOR
        printf("reset to Factory failed! \n");
        #endif
    }


    return ret;
#endif


    ret=execresetFactory((char*)"/mnt/spiflashtxt/backup/image_process", (char*)"/mnt/spiflashtxt/backup/imageprocess.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/image_process /mnt/spiflashtxt/  && cp /mnt/spiflashtxt/backup/image_process /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/image_process /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/image_process /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/imageprocess.txt.md5 /mnt/spiflashtxt/  && cp /mnt/spiflashtxt/backup/imageprocess.txt.md5 /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/imageprocess.txt.md5 /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/imageprocess.txt.md5 /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy image_process success\n");
        #endif

    }



    ret=execresetFactory((char*)"/mnt/spiflashtxt/backup/monitor", (char*)"/mnt/spiflashtxt/backup/monitor.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/monitor /mnt/spiflashtxt/  && cp /mnt/spiflashtxt/backup/monitor /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/monitor /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/monitor /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/monitor.txt.md5 /mnt/spiflashtxt/  && cp /mnt/spiflashtxt/backup/monitor.txt.md5 /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/monitor.txt.md5 /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/monitor.txt.md5 /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy monitor success\n");
        #endif

    }


    ret=execresetFactory((char*)"/mnt/spiflashtxt/backup/poly4_S_HYG_full_smag55_dangle-35.txt", (char*)"/mnt/spiflashtxt/backup/poly4.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/poly4_S_HYG_full_smag55_dangle-35.txt  /mnt/spiflashtxt/config  && cp /mnt/spiflashtxt/backup/poly4_S_HYG_full_smag55_dangle-35.txt /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/poly4_S_HYG_full_smag55_dangle-35.txt /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/poly4_S_HYG_full_smag55_dangle-35.txt /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/poly4.txt.md5 /mnt/spiflashtxt/config  && cp /mnt/spiflashtxt/backup/poly4.txt.md5 /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/poly4.txt.md5 /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/poly4.txt.md5 /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy poly4 success\n");
        #endif

    }



    ret=execresetFactory((char*)"/mnt/spiflashtxt/backup/Shanghai-804-Satallite-20210511-ID.prj", (char*)"/mnt/spiflashtxt/backup/Shanghai.txt.md5");
    if(ret==0)
     {
        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/Shanghai-804-Satallite-20210511-ID.prj  /mnt/spiflashtxt/data  && cp /mnt/spiflashtxt/backup/Shanghai-804-Satallite-20210511-ID.prj /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/Shanghai-804-Satallite-20210511-ID.prj /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/Shanghai-804-Satallite-20210511-ID.prj /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
            count++;
        }

        ret=ExecuteCMD((char*)"cp /mnt/spiflashtxt/backup/Shanghai.txt.md5 /mnt/spiflashtxt/data  && cp /mnt/spiflashtxt/backup/Shanghai.txt.md5 /mnt/spiflashtxt/usr_process_1  && cp /mnt/spiflashtxt/backup/Shanghai.txt.md5 /mnt/spiflashtxt/usr_process_2 &&cp /mnt/spiflashtxt/backup/Shanghai.txt.md5 /mnt/spiflashtxt/usr_process_3",result);
        if(ret==0)
        {
           count++;
        }

        #ifdef IF_PRINTF_MONITOR
        printf("copy Shanghai-804-Satallite-20210511-ID.prj success\n");
        #endif

    }


    if(count==8)
    {
        ret=0;
    }
    else
    {
        ret=-1;
    }
    #ifdef IF_PRINTF_MONITOR
    printf("ret  count %d\n",count);
    #endif
    return ret;



}


/*****************************************************
 * @brief 根据程序名，获取进程的PID号
 * 遍历/proc下的进程，如果/proc/stat下的程序名匹配，则返回进程号
 * 如果程序不在运行，返回-1
 * @param name
 * @return pid_t pid的值
 ********************************************************/
pid_t getProcessPID(const char* name)
{
    DIR *dir = NULL;
    struct dirent *d = NULL;
    pid_t pid = 0;
    dir = opendir("/proc");
    if (!dir)
    {
        #ifdef  IF_PRINTF_MONITOR
        printf("cannot open /proc.\n");
        #endif
        return -1;
    }

    while((d = readdir(dir)) != NULL)
    {
        if((pid = atoi(d->d_name)) == 0)
        {
            continue;
        }
        char base_path[100] = "/proc/";
        strcat(base_path, d->d_name);
        strcat(base_path, "/stat");
        FILE* fd = fopen(base_path, "r");
        char stat_name[15] = {0};
        char tmp;
        while(fgetc(fd) != '(')
        {
            continue;
        }
        int i = 0;
        while((tmp = fgetc(fd)) != ')')
        {
            stat_name[i++] = tmp;
        }
        if(!strcmp(stat_name, name)) //如果名称相同
        {
            #ifdef  IF_PRINTF_MONITOR
            printf("The PID of %s is %d \n", name, pid);
            #endif
            return pid;
        }
    }
    #ifdef  IF_PRINTF_MONITOR
    printf("can not get pid, %s isn't running? \n", name);
    #endif
    return -1;
}

int startProcess(const char* process_name)
{
    pid_t pid = fork();
    if(pid < 0)
    {
        perror("fork:");
        return -1;
    }
    else if(pid == 0)
    {
        char path[100] = "/spiflash/";
        //char path[100] = "/mnt/logzg/lib_dir/";
        strcat(path, process_name);
        char tmp[100] = {0};
        strcpy(tmp, process_name);
        char* const args[] = {tmp, NULL};
        execv(path, args);
    }
    return 0;
}
/***************************************************
 * brief 根据进程名重启进程
 *
 * param process_name 进程名字符串
 * return int
 ******************************************************/
int resetProcess(const char* process_name)
{
    //获取进程pid pidof()
    pid_t process_pid = getProcessPID(process_name);
    //杀死进程
    if(process_pid > 0)
    {
        kill(process_pid, SIGTERM);
        waitpid(process_pid, NULL, WNOHANG);
        //重启进程
        #ifdef  IF_PRINTF_MONITOR
        printf("restart %s ... \n", process_name);
        #endif
        startProcess(process_name);
        return 1;
    }
    else
    {
       startProcess(process_name);
       return 1;
    }
    return 0;
}







void InitBramMonitor()
{
    initBram(&pBramInfo[0],SHARE_YC,BRAM_LENTH);
}

//初始化时钟的BRAM
void InitBramClock()
{

    initBram(&pBramInfo[2],SHARE_CLOCK,BRAM_LENTH_CLOCK);
}

//i==0 20M  i==1 10M
void SetClockFre(unsigned int flag)
{

  #if 1
    //0x200
    unsigned char data1[4];

    readBram(&pBramInfo[2],data1,0x200,4);
    #ifdef  IF_PRINTF_MONITOR
    printf("clock_0x200:%x %x %x %x\n",data1[0],data1[1],data1[2],data1[3]);
    #endif

    //2倍频

    memset(data1,0,4);
    data1[0]=0x01;
    data1[1]=0x02;
    //data1[2]=0x01;
    //data1[3]=0x01;
    writeBram(&pBramInfo[2],data1,0x200,4);


    unsigned char data[4];
    //0x208
    memset(data,0,4);//video0传输的是sdi1
    if(flag==0) //20M
    {
        data[0]=2;
        writeBram(&pBramInfo[2],data,0x208,4);

    }
    else
    {
        data[0]=4;
        writeBram(&pBramInfo[2],data,0x208,4);
    }

    //0x04
    //unsigned char data1[4];
    //readBram(&pBramInfo[2],data1,0x04,4);

    usleep(500*1000);
    //启动 //0x25c
    unsigned char data2[4];
    memset(data2,0,4);//video0传输的是sdi1
    //readBram(&pBramInfo[2],data2,0x25c,4);
    //data2[0]&=0xfe;
    data2[0]=0x3;
    writeBram(&pBramInfo[2],data2,0x25c,4);
   #endif

    /*
    unsigned char data2[4];
    memset(data2,0,4);//video0传输的是sdi1
    //readBram(&pBramInfo[2],data2,0x25c,4);
    //data2[0]&=0xfe;
    data2[0]=0x1;
    writeBram(&pBramInfo[2],data2,0x25c,4);
   */


}


#ifndef HandCom
void SetWorkmode(unsigned short &data_in)
{
    //根据mode_change[i]和mode_free[i]来更新工作模式这个遥测量
    unsigned int i;
    unsigned short mode=0;
    unsigned int flag=0;


    if(mode_lx==1)
    {

        for(i=0;i<6;i++)
        {

            flag=mode_if_changesend[i];

            if(i<4)  //在video0 和video1所在位置
             {

                if(mode_if_free[i]==0) //待机
                {
                    mode|=((1&0x3)<<(2*flag));
                }
                else if(mode_if_free[i]==1) //测量
                {
                    mode|=((2&0x3)<<(2*flag));
                }
            }
            else
            {

                if(mode_if_free[i]==0) //待机
                {
                    mode|=((1&0x3)<<(2*flag));
                }
                else if(mode_if_free[i]==1) //件事
                {
                    mode|=((3&0x3)<<(2*flag));
                }

            }
         }
    }
    else
    {
        for(i=0;i<6;i++)
        {

            flag=mode_if_changesend[i];

            if(i<2)  //在video0 和video1所在位置
             {

                if(mode_if_free[flag]==0) //待机
                {
                    mode|=((1&0x3)<<(2*flag));
                }
                else if(mode_if_free[flag]==1) //测量
                {
                    mode|=((2&0x3)<<(2*flag));
                }
            }
            else
            {

                if(mode_if_free[flag]==0) //待机
                {
                    mode|=((1&0x3)<<(2*flag));
                }
                else if(mode_if_free[flag]==1) //监控
                {
                    mode|=((3&0x3)<<(2*flag));
                }

            }
         }
    }
    data_in=mode;

    #ifdef  USE_PRINTF

    printf("mode_if_free %d %d %d %d %d %d \n",mode_if_free[0],mode_if_free[1],mode_if_free[2],mode_if_free[3],mode_if_free[4],mode_if_free[5]);
    //printf("mode_if_change %d %d %d %d %d %d \n",mode_if_change[0],mode_if_change[1],mode_if_change[2],mode_if_change[3],mode_if_change[4],mode_if_change[5]);
    printf("mode_if_change %d %d %d %d %d %d \n",mode_if_changesend[0],mode_if_changesend[1],mode_if_changesend[2],mode_if_changesend[3],mode_if_changesend[4],mode_if_changesend[5]);
    printf("work_mode is %x\n",data_in);
     #endif
}
#endif

#ifdef HandCom

void SetWorkmode(unsigned short &data_in)
{
    //根据mode_change[i]和mode_free[i]来更新工作模式这个遥测量
    unsigned int i;
    unsigned short mode=0;
    unsigned int flag=0;


    if(mode_lx==1)
    {

        for(i=0;i<6;i++)
        {

            flag=mode_if_changesend[i];

            if(i<4)  //在video0 和video1所在位置
             {

               mode|=((2&0x3)<<(2*flag));

            }
            else
            {
               mode|=((3&0x3)<<(2*flag));

            }
         }
    }
    else
    {
        for(i=0;i<6;i++)
        {

            flag=mode_if_changesend[i];

            if(i<2)  //在video0 和video1所在位置
             {
               mode|=((2&0x3)<<(2*flag));
            }
            else
            {
               mode|=((3&0x3)<<(2*flag));
            }
         }
    }
    data_in=mode;

    #ifdef  USE_PRINTF
    printf("mode_if_free %d %d %d %d %d %d \n",mode_if_free[0],mode_if_free[1],mode_if_free[2],mode_if_free[3],mode_if_free[4],mode_if_free[5]);
    //printf("mode_if_change %d %d %d %d %d %d \n",mode_if_change[0],mode_if_change[1],mode_if_change[2],mode_if_change[3],mode_if_change[4],mode_if_change[5]);
    printf("mode_if_change %d %d %d %d %d %d \n",mode_if_changesend[0],mode_if_changesend[1],mode_if_changesend[2],mode_if_changesend[3],mode_if_changesend[4],mode_if_changesend[5]);
    printf("work_mode is %x\n",data_in);
    #endif
}
#endif


void ModifyFjyk()
{

   unsigned char data[8];
   memset(data,0,8);//video0传输的是sdi1

   data[3]=(mode_if_changesend[2]+1);
   data[2]=(mode_if_changesend[2]+1);
   data[0]=(mode_if_changesend[3]+1);
   data[1]=(mode_if_changesend[3]+1);

   data[7]=(mode_if_changesend[4]+1);
   data[6]=(mode_if_changesend[4]+1);
   data[5]=(mode_if_changesend[5]+1);
   data[4]=(mode_if_changesend[5]+1);

   writeBram(&pBramInfo[0],data,0x0,8);


   unsigned char data1[8];
   readBram(&pBramInfo[0],data1,0,8);

   #ifdef IF_PRINTF_MONITOR
   printf("data1 %x %x %x %x %x %x %x %x\n",data1[0],data1[1],data1[2],data1[3],data1[4],data1[5],data1[6],data1[7]);
   #endif
}



//反馈遥控指令正确与否的缓存
char feedback_data[7]={(char)0xeb,(char)0x90,(char)0xe1,0x00,0x1,0x00,0x00};

#if 0
FILE *fd_yc;
unsigned flag_write=0;
#endif
unsigned char checkSum(unsigned char cmd[], int len, unsigned char checkSum)
{
    unsigned char sum = 0;

    #if 0
    if(flag_write==0)
    {
     fd_yc = fopen("/mnt/ycsend.dat", "wb");
     flag_write=1;
    }
    #endif

    for (int index = 0; index < len; index++)
    {
        sum += cmd[index];
    }

    if (sum == checkSum)
    {

        return SUCCESS;
    }
    else
    {
        #ifdef IF_PRINTF_MONITOR
        printf("校验和错误sum :%x len:%d checkSum:%x\n",sum,len,checkSum);
        #endif

        #if 0
        //写文件
        fwrite((char*)cmd-6, len+7, 1, fd_yc);
        //fwrite((char*)&checkSum, 1, 1, fd_yc);
        fflush(fd_yc);
        #endif

        //printf("checkSum:%x sum:%x\n",checkSum);
        return ERROR;
    }
}

unsigned char checkSum1(unsigned char cmd[], int len, unsigned char checkSum)
{
    unsigned char sum = 0;

    #if 0
    if(flag_write==0)
    {
     fd_yc = fopen("/mnt/ycsend.dat", "wb");
     flag_write=1;
    }
    #endif

    for (int index = 0; index < len; index++)
    {
        sum += cmd[index];
    }

    if (sum == checkSum)
    {

        return SUCCESS;
    }
    else
    {
        #ifdef IF_PRINTF_MONITOR
        printf("图像校验和错误sum :%x len:%d checkSum:%x\n",sum,len,checkSum);
        #endif


        #if 0
        //写文件
        fwrite((char*)cmd-6, len+7, 1, fd_yc);
        //fwrite((char*)&checkSum, 1, 1, fd_yc);
        fflush(fd_yc);
        #endif

        //printf("checkSum:%x sum:%x\n",checkSum);
        return ERROR;
    }
}

/*
void SendImageSoftWare(unsigned char *data,unsigned int cmd_data)
{
    unsigned char sum=0;
    unsigned int i;
    data[0]=0xeb;
    data[1]=0x90;
    data[2]=0x11; //指令类型码
    data[3]=0x0;
    data[4]=0x02;
    data[5]=(unsigned char)(cmd_data);
    data[6]=(unsigned char)(cmd_data>>8);
    for(i=0;i<7;i++)
    {
        sum+=data[i];
    }
    data[7]=sum;

}
*/
/****************************************************************************************************
 * 2 网络发送数据
 *
 *
 * ***************************************************************************************************/
void SendImageSoftWare(unsigned char *data,unsigned char * data_in,unsigned char lenth,unsigned char mode)
{
    unsigned char sum=0;
    unsigned int i;
    memset(data,0x00,send_net_number);

    data[0]=0xeb;
    data[1]=0x90;
    data[2]=mode; //指令类型码
    data[3]=lenth;
    memcpy(data+4,data_in,lenth);

    for(i=0;i<send_net_number;i++)
    {
        sum+=data[i];
    }
    data[send_net_number-1]=sum;

}


/*
int SendUartIns(int fd_send,unsigned char *data,unsigned char * data_in,unsigned char lenth,unsigned char mode)
{

    unsigned char sum=0;
    unsigned int i;
    int status;
    memset(data,0x00,send_uart_number);

    data[0]=0xeb;
    data[1]=0x90;
    data[2]=mode; //指令类型码
    //data[3]=lenth;
    memcpy(data+3,data_in,lenth);

    for(i=0;i<(send_uart_number-1);i++)
    {
        sum+=data[i];
    }
    data[send_uart_number-1]=sum;

    status=UART0_Send(fd_send,(char *)data,send_uart_number);
    return status;

}
*/
/*************************************************************************
 * 1.flag=0  自动曝光开
 *
 *
 *
 *
 *
 *
 *
 * ****************************************************************************/
#ifndef IF_EXPOSURE
int SendUartIns(int fd_send,unsigned int flag)
{

    unsigned char sum=0;
    unsigned int i;
    int status;
    memset(send_uart_camera,0x00,send_uart_number);

    send_uart_camera[0]=0xeb;
    send_uart_camera[1]=0x90;
    //data[2]=mode; //指令类型码
    //data[3]=lenth;
    //memcpy(data+2,data_in,lenth);

    if(flag==0)  //自动曝光开
    {
        send_uart_camera[2]=0xbb;
        send_uart_camera[3]=0x01;
    }
    else if(flag==1)  //自动曝光关
    {
        send_uart_camera[2]=0xbb;
        send_uart_camera[3]=0x02;
    }
    else if(flag==2) //告警
    {
        send_uart_camera[2]=0xbc;
        send_uart_camera[3]=0x01;
    }
    else if(flag==3) //监控
    {
        send_uart_camera[2]=0xbc;
        send_uart_camera[3]=0x02;
    }
    else if(flag==4) //待机
    {
        send_uart_camera[2]=0xba;
        send_uart_camera[3]=0x01;
    }
    else if(flag==5) //退出待机
    {
        send_uart_camera[2]=0xba;
        send_uart_camera[3]=0x02;
    }
    send_uart_camera[4]=0x00;
    send_uart_camera[5]=0x11;
    send_uart_camera[6]=0x22;
    send_uart_camera[7]=0x33;

    for(i=0;i<(send_uart_number-1);i++)
    {
        sum+=send_uart_camera[i];
    }
    send_uart_camera[send_uart_number-1]=sum;


    #ifdef IF_PRINTF_MONITOR
    printf("comdatayk:%x %x %x %x %x %x %x %x %x\n",send_uart_camera[0],send_uart_camera[1],send_uart_camera[2],send_uart_camera[3],send_uart_camera[4],send_uart_camera[5],send_uart_camera[6],send_uart_camera[7],send_uart_camera[8]);
    #endif
    status=UART0_Send(fd_send,(char *)send_uart_camera,send_uart_number);
    return status;

}

int SendUartInszs(int fd_send,unsigned char *data,unsigned char * data_in,unsigned int lenth)
{

    unsigned char sum=0;
    unsigned int i;
    int status;
    memset(data,0x00,send_uart_number);

    data[0]=0xeb;
    data[1]=0x90;
    data[2]=0xcf;
    data[3]=0x01;
    //data[2]=mode; //指令类型码
    //data[3]=lenth;
    memcpy(data+4,data_in,lenth);



    for(i=0;i<(send_uart_number-1);i++)
    {
        sum+=data[i];
    }
    data[send_uart_number-1]=sum;




    status=UART0_Send(fd_send,(char *)data,send_uart_number);
    #ifdef IF_PRINTF_MONITOR
    printf("comdata:%x %x %x %x %x %x %x %x %x\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
    #endif
    return status;

}
#endif



#ifdef IF_EXPOSURE
int SendUartIns(int fd_send,unsigned int flag)
{

    unsigned char sum=0;
    unsigned int i;
    int status;
    memset(send_uart_camera,0x00,send_uart_number);

    send_uart_camera[0]=0xeb;
    send_uart_camera[1]=0x90;
    //data[2]=mode; //指令类型码
    //data[3]=lenth;
    //memcpy(data+2,data_in,lenth);

    if(flag==0)  //自动曝光开
    {
        send_uart_camera[2]=0xbb;
        send_uart_camera[3]=0x01;
    }
    else if(flag==1)  //自动曝光关
    {
        send_uart_camera[2]=0xbb;
        send_uart_camera[3]=0x02;
    }
    else if(flag==2) //告警
    {
        send_uart_camera[2]=0xbc;
        send_uart_camera[3]=0x01;
    }
    else if(flag==3) //监控
    {
        send_uart_camera[2]=0xbc;
        send_uart_camera[3]=0x02;
    }
    else if(flag==4) //待机
    {
        send_uart_camera[2]=0xba;
        send_uart_camera[3]=0x01;
    }
    else if(flag==5) //退出待机
    {
        send_uart_camera[2]=0xba;
        send_uart_camera[3]=0x02;
    }
    send_uart_camera[4]=0x00;
    send_uart_camera[5]=0x11;
    send_uart_camera[6]=0x22;
    send_uart_camera[7]=0x33;

    send_uart_camera[8]=0;
    send_uart_camera[9]=0;
    send_uart_camera[10]=0;

    for(i=0;i<(send_uart_number-1);i++)
    {
        sum+=send_uart_camera[i];
    }


    send_uart_camera[send_uart_number-1]=sum;


    #ifdef IF_PRINTF_MONITOR
    printf("comdatayk:%x %x %x %x %x %x %x %x %x\n",send_uart_camera[0],send_uart_camera[1],send_uart_camera[2],send_uart_camera[3],send_uart_camera[4],send_uart_camera[5],send_uart_camera[6],send_uart_camera[7],send_uart_camera[8]);
    #endif
    status=UART0_Send(fd_send,(char *)send_uart_camera,send_uart_number);
    return status;

}

int SendUartInszs(int fd_send,unsigned char *data,unsigned char * data_in,unsigned int lenth)
{

    unsigned char sum=0;
    unsigned int i;
    int status;
    memset(data,0x00,send_uart_number);

    data[0]=0xeb;
    data[1]=0x90;
    data[2]=0xcf;
    data[3]=0x01;
    //data[2]=mode; //指令类型码
    //data[3]=lenth;
    memcpy(data+4,data_in,lenth);


    data[8]=0;
    data[9]=0;
    data[10]=0;

    for(i=0;i<(send_uart_number-1);i++)
    {
        sum+=data[i];
    }

    data[send_uart_number-1]=sum;



    status=UART0_Send(fd_send,(char *)data,send_uart_number);
    #ifdef IF_PRINTF_MONITOR
    printf("comdata:%x %x %x %x %x %x %x %x %x\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
    #endif
    return status;

}
#endif


/***********************************************************
 * 1.遥控指令解析
 * 2.反馈正确与否的指令
 * ********************************************************/
void telctl_feedback(int fd_send,char flag_send,char ins_mode)
{

    feedback_data[2]=ins_mode;
    feedback_data[5]=flag_send;

    feedback_data[6]=(char)(cmputCheckSum((unsigned char*)(feedback_data+2),4));


    UART0_Send(fd_send,feedback_data,7);
}



void InitToFpga(bramInfo *m_bramInfodata,unsigned int baseAddr,int memsize)
{
    int status=initBram(m_bramInfodata,baseAddr,memsize);
    if(status==-1)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("init bram failed");
        #endif
    }
}
/**************************************************************
 * 1.计算校验和
 *
 *
 *
 *
 * ***********************************************************/
unsigned char cmputCheckSum(unsigned char data[],int len)
{
    unsigned char sum = 0;
    for(int i = 0;i < len;i++)
    {
        sum += data[i];
    }
    return sum;
}

unsigned int  InitTmData(tmPackage *tmdatatemp)
{
    unsigned char *data;
    int lenth=sizeof(tmPackage);


    memset(tmdatatemp,sizeof(tmPackage),0);
    tmdatatemp->Frame_Header=0xeb90;
    tmdatatemp->Typeins=0xf0;
    tmdatatemp->Mutiple_lenth=lenth-8;
    tmdatatemp->EPDU_Lenth=(lenth-6);  //有效epdu字节数
    tmdatatemp->Packet_Identity=0x0107;
    tmdatatemp->Packet_Control=(0x3<<14);

    //测试用
    //tmdatatemp->Packet_Control|=0x3ff0;
    tmdatatemp->Packet_Lenth=lenth-15;
    //tmdatatemp->Packet_Control=0xc000;
    data=((unsigned char *)tmdatatemp)+2;
    tmdatatemp->sum=cmputCheckSum(data,lenth-3);
    tmdatatemp->data_out[0]=0x27;

    #ifdef IF_PRINTF_MONITOR
    printf("tmdatatemp->Mutiple_lenth:%d tmdatatemp->Packet_Lenth:%d tmdatatemp->EPDU_Lenth:%d\n",tmdatatemp->Mutiple_lenth,tmdatatemp->Packet_Lenth,tmdatatemp->EPDU_Lenth);
    #endif
   // tmdatatemp->widearea_a_workmode=2;
   // tmdatatemp->widearea_b_workmode=2;
    //tmdatatemp->widearea_c_workmode=2;
    //tmdatatemp->widearea_d_workmode=2;

    tmdatatemp->ins_count=0;
    tmdatatemp->wrong_ins_count=0;

    #ifdef IF_PRINTF_MONITOR
    printf("lenth:%d EPDU_Lenth%d  Packet_Control:%x\n ",lenth,tmdatatemp->EPDU_Lenth,tmdatatemp->Packet_Control);
    #endif

    return  (tmdatatemp->EPDU_Lenth+6);

}

unsigned int  SendTmData(tmPackage *tmdatatemp)
{
    //unsigned char *data;

    unsigned int fj_data_temp[4];
    unsigned int i;


    //将数据复制进来
    //将图像复接的遥测读取处理
    readBram(&pBramInfo[0],(unsigned char*)fj_data_temp,0x8,16);
    memcpy(tmdatatemp->fj_data,fj_data_temp,3*sizeof(unsigned int));
    for(i=0;i<3;i++)
    {
       tmdatatemp->fj_data[i]=ntohl(tmdatatemp->fj_data[i]);
     }
    //将状态复制进去


    tmdatatemp->thread_flag[4]&=0xf0;
    tmdatatemp->thread_flag[4]|=((thread_flag_tj[0]&0xf));
    tmdatatemp->thread_flag[5]=(((thread_flag_tj[1]&0xf)<<4)|(thread_flag_tj[2]&0xf));
    tmdatatemp->thread_flag[6]=(((thread_flag_tj[3]&0xf)<<4)|(thread_flag_tj[4]&0xf));    
    tmdatatemp->thread_flag[7]&=0xf;
    tmdatatemp->thread_flag[7]|=((thread_flag_tj[5]&0xf)<<4);
    //memcpy(&(tmdatatemp->thread_flag[Image_Thread_NUMBER]),thread_flag_tj,Moniter_Thread_NUMBER);  //2023.5.4

    tmdatatemp->version=log_data.version_imageprocess;

    //将工作状态黏贴进去
   //memcpy(&(tmdatatemp->thread_flag[Image_Thread_NUMBER+Moniter_Thread_NUMBER]),&work_mode_tj,1); //2023.5.4



    tmdatatemp->status_zt&=(~(0x80));
    tmdatatemp->status_zt|=((sc_on_off<<7)&0x80);  //bit7
    tmdatatemp->status_zt&=(~(0x60));
    tmdatatemp->status_zt|=((zg_status<<5)&0x60);  //bit6~bit5
    tmdatatemp->status_zt&=(~(0x18));
    tmdatatemp->status_zt|=((work_mode_tj<<3)&0x18); //bit4~bit3
    tmdatatemp->status_zt&=(~(0x4));
    tmdatatemp->status_zt|=((bitmap_success<<2)&4);//bit2
    tmdatatemp->status_zt&=(~(0x2));//bit1
    tmdatatemp->status_zt|=((zg_excut<<1)&2);//bit1
    tmdatatemp->status_zt&=(~(0x1));//bit0
    tmdatatemp->status_zt|=(clock_status);//bit0

    tmdatatemp->zg_rightcount=log_data.zg_rightcount;
    tmdatatemp->zg_wrongcount=log_data.zg_wrongcount;
    tmdatatemp->bitmap_zg_rightcount=log_data.bitmap_zg_rightcount;  //2023.11.3

   // printf("tmdatatemp->status_zt:%x\n",tmdatatemp->status_zt);

    //校验和

    //data=((unsigned char *)tmdatatemp)+2;

    tmdatatemp->Packet_Control++;

    tmdatatemp->Packet_Control&=0x3fff;
    tmdatatemp->Packet_Control|=0xc000;
    /*

    if(((tmdatatemp->Packet_Control)&0x3fff)==0x0)
    {
        tmdatatemp->Packet_Control=0xc000;
    }
    */






    /*
    unsigned int n=(sizeof(tmPackage)-3);
    for(unsigned i=0;i<n;i++)
    {
        data[i]=i;
    }
    tmdatatemp->sum=cmputCheckSum(data,(sizeof(tmPackage)-3));
    */

    //tmdatatemp->sum=cmputCheckSum(data,(sizeof(tmPackage)-3));




    //高低字节转换
    return  (tmdatatemp->EPDU_Lenth+6);

}



//需要调用此函数多少此

int SendBitMapYcCount()
{

    bitmap_lenth=getBitMap_BlockData(BITMAP_ROUTE, sizeof(BITMAP_ROUTE), yc_bitmap_data);
    if(bitmap_lenth<0)
    {
       return -1;
    }
    bitmap_mode=bitmap_lenth/238;
    bitmap_ys=bitmap_lenth%238;
    if(bitmap_ys>0)
    {
         bitmap_zj=(bitmap_mode+1);
    }
    else
    {
        bitmap_zj=bitmap_mode;
    }


    return bitmap_zj;

}
int SendBitMapYc(tmPackage *tmdata,unsigned char *data)
{

    unsigned int i;
    //将yc复制进来
    if(bitmap_setcount<bitmap_zj)
    {
       memcpy(yc_send,tmdata,sizeof(tmPackage)-1);  //除掉校验和
       //构造bitmapfile
       yc_bitmap_data_file[0]=0x01;
       yc_bitmap_data_file[1]=0x0f;

       bitmap_epfucount++;

       /*
       if(((bitmap_epfucount)&0x3fff)==0x0)
       {
           bitmap_epfucount=0x8000;
       }
       */


       bitmap_epfucount&=0x3fff;
       bitmap_epfucount|=0xc000;


       yc_bitmap_data_file[2]=(bitmap_epfucount>>8);
       yc_bitmap_data_file[3]=(bitmap_epfucount&0xff);

       if(bitmap_setcount==(bitmap_zj-1))
        {
           yc_bitmap_data_file[4]=0;
           yc_bitmap_data_file[5]=(bitmap_ys+12-1);
           yc_bitmap_data_file[6]=0x3; //和校验
           yc_bitmap_data_file[7]=0x2; //bitmap表
           yc_bitmap_data_file[8]=0xaa; //文件标识
           yc_bitmap_data_file[9]=0xbb; //文件标识
           yc_bitmap_data_file[10]=0;
           yc_bitmap_data_file[11]=0;
           yc_bitmap_data_file[12]=0;
           yc_bitmap_data_file[13]=(bitmap_ys+12);  //从版本号到校验码
           memcpy(yc_bitmap_data_file+14,yc_bitmap_data+bitmap_setcount*238,bitmap_ys);

           for(i=6;i<(bitmap_ys+14);i++)
           {
               sum_bitmap+=yc_bitmap_data_file[i];
           }

           (*(yc_bitmap_data_file+bitmap_ys+8+6))=(sum_bitmap>>24);
           (*(yc_bitmap_data_file+bitmap_ys+8+6+1))=(sum_bitmap>>16);
           (*(yc_bitmap_data_file+bitmap_ys+8+6+2))=(sum_bitmap>>8);
           (*(yc_bitmap_data_file+bitmap_ys+8+6+3))=(sum_bitmap);



           yc_bitmap_data_file_lenth=(bitmap_ys+12+6);

       }
       else
       {

           yc_bitmap_data_file[4]=0;
           yc_bitmap_data_file[5]=249;
           yc_bitmap_data_file[6]=0x3; //无校验
           yc_bitmap_data_file[7]=0x2; //bitmap表
           yc_bitmap_data_file[8]=0xaa; //文件标识
           yc_bitmap_data_file[9]=0xbb; //文件标识
           yc_bitmap_data_file[10]=0;
           yc_bitmap_data_file[11]=0;
           yc_bitmap_data_file[12]=0;
           yc_bitmap_data_file[13]=250;
           memcpy(yc_bitmap_data_file+14,yc_bitmap_data+bitmap_setcount*238,238);

           for(i=6;i<252;i++)
           {
               sum_bitmap+=yc_bitmap_data_file[i];
           }

           (*(yc_bitmap_data_file+252))=(sum_bitmap>>24);
           (*(yc_bitmap_data_file+253))=(sum_bitmap>>16);
           (*(yc_bitmap_data_file+254))=(sum_bitmap>>8);
           (*(yc_bitmap_data_file+255))=(sum_bitmap);

//                      (*(yc_bitmap_data_file+252))=0xaa;
//                      (*(yc_bitmap_data_file+253))=0xbb;
//                      (*(yc_bitmap_data_file+254))=0xcc;
//                      (*(yc_bitmap_data_file+255))=0xdd;



           yc_bitmap_data_file_lenth=256;
       }

       memcpy(yc_send+sizeof(tmPackage)-1,yc_bitmap_data_file,yc_bitmap_data_file_lenth);



       //构造两个epdu
       EPDU_LenthAddBitmap=(sizeof(tmPackage)+yc_bitmap_data_file_lenth-6);
       Mutiple_lenthAddBitmap=(sizeof(tmPackage)+yc_bitmap_data_file_lenth-8);

       yc_send[3]=(EPDU_LenthAddBitmap>>8);
       yc_send[4]=EPDU_LenthAddBitmap&0xff;
       yc_send[5]=(Mutiple_lenthAddBitmap>>8);
       yc_send[6]=Mutiple_lenthAddBitmap&0xff;

       //变为首包
       yc_send[9]&=0x3f;
       yc_send[9]|=(1<<6);





       sum_epdu=0;
       //计算校验和
       for(i=2;i<(sizeof(tmPackage)+yc_bitmap_data_file_lenth-1);i++)
        {
           sum_epdu+=yc_send[i];
       }

      // printf("bitmapsend:%d\n",(sizeof(tmPackage)+yc_bitmap_data_file_lenth));

       yc_send[sizeof(tmPackage)+yc_bitmap_data_file_lenth-1]=sum_epdu;

       bitmap_setcount++;
       return sizeof(tmPackage)+yc_bitmap_data_file_lenth;
    }

}
/*****************************************************************
 *1.bram方式发送指令
 *
 *
 * ***************************************************************/
void sendParamToFPGA(bramInfo *m_bramInfodata,unsigned char *data,int offset,int len)
{
     int status=writeBram(m_bramInfodata,data,offset,len);
     if(status==-1)
     {
         #ifdef IF_PRINTF_MONITOR
         printf("write bram failed");
         #endif
     }
}

/***********************************************************
 *1.给相机发送不同的指令
 * 0xeb90
 * 指令类型 mode_in
 *
 *
 *
 *
 *
 *
 ************************************************************/
/*
int SendUartIns(int fd_send, int send_in)
{
    char send_data[100];
    int status;
    send_data[0]=0xeb;
    send_data[1]=0x90;
    memcpy(send_data+2,&send_in,4);

    status=UART0_Send(fd_send,send_data,20);
    return status;
}
*/

/*
void ZhByteFour(unsigned char *temp)
{

    unsigned char temp1;
    temp1=temp[0];
    temp[0]=temp[3];
    temp[3]=temp1;
    temp1=temp[1];
    temp[2]=temp[1];
    temp[1]=temp1;
}
*/


/****************************************************************
 * ntohl  ntohs
 *
 *
 * **************************************************************/
void ZhData(tmPackage *data_in,tmPackage *data_out)
{

    //static unsigned int temp;
    //int aa=sizeof(tmPackage);
    //unsigned int i;
    //unsigned char *data;

   // unsigned int fj_data_temp[4];

    //static unsigned char aaa=0;

#if 0
    static unsigned long time_data1;
    static unsigned long time_data1_temp;
    float time_data1cz;
    static unsigned time_data1_flag=0;


      time_data1=((data_in->target_frame_data.timesamp[0]<<24)|(data_in->target_frame_data.timesamp[1]<<16)|(data_in->target_frame_data.timesamp[2]<<8)|(data_in->target_frame_data.timesamp[3]))*10000+\
              ((data_in->target_frame_data.timesamp[4]<<8)|data_in->target_frame_data.timesamp[5]);
     // printf("时间send：%x %x %x %x %x %x\n",data_in->target_frame_data.timesamp[0],data_in->target_frame_data.timesamp[1],\
              //data_in->target_frame_data.timesamp[2],data_in->target_frame_data.timesamp[3],\
             // data_in->target_frame_data.timesamp[4],data_in->target_frame_data.timesamp[5]);
      if(time_data1_flag==1)
      {
          time_data1cz= ((time_data1- time_data1_temp)/10000.);
          if(time_data1cz>1)
           {
              printf("wrong chazhi  time_data1cz:%f\n",time_data1cz);
           }
          //printf("time_data1cz:%f\n",time_data1cz);


      }
      time_data1_temp=time_data1;
      time_data1_flag=1;

#endif



    //将workmode模式计算出
    //SetWorkmode((*data_in).work_mode);



    //拷贝
    memcpy(data_out,data_in,sizeof(tmPackage));


    data_out->Frame_Header=ntohs(data_in->Frame_Header);  //short
    data_out->Packet_Identity=ntohs(data_in->Packet_Identity); //short
    data_out->EPDU_Lenth=ntohs(data_in->EPDU_Lenth); //short
    data_out->Packet_Control=ntohs(data_in->Packet_Control);
    data_out->Mutiple_lenth=ntohs(data_in->Mutiple_lenth);

    data_out->Packet_Lenth=ntohs(data_in->Packet_Lenth);



    //data_out->work_mode=ntohs(data_in->work_mode); //short

    (data_out->point_frame_data).farcamera_bodyAzu=ntohs((data_in->point_frame_data).farcamera_bodyAzu);
    (data_out->point_frame_data).farcamera_bodyPit=ntohs((data_in->point_frame_data).farcamera_bodyPit);
    //(data_out->point_frame_data).farcamera_Alarm_staus1=0xbb;//0xbb删除根据实际情况做操作
    (data_out->point_frame_data).farcamera_used_time1=ntohs((data_in->point_frame_data).farcamera_used_time1); //short

//    aaa++;
//    (data_out->point_frame_data).farcamera_timesamp1[0]=aaa;


    //(data_out->point_frame_data).farcamera_distance=ntohl((data_in->point_frame_data).farcamera_distance);
    //(data_out->point_frame_data).farcamera_used_time1=ntohs((data_in->point_frame_data).farcamera_used_time1);
    (data_out->point_frame_data).bodyAzu1_0=ntohs((data_in->point_frame_data).bodyAzu1_0); //short
    (data_out->point_frame_data).bodyAzu1_1=ntohs((data_in->point_frame_data).bodyAzu1_1);
    //(data_out->point_frame_data).bodyAzu1_2=ntohs((data_in->point_frame_data).bodyAzu1_2); //2023.9.11
    (data_out->point_frame_data).bodysize1_0=ntohs((data_in->point_frame_data).bodysize1_0); //2023.9.11
    (data_out->point_frame_data).bodyPit1_0=ntohs((data_in->point_frame_data).bodyPit1_0); //short
    (data_out->point_frame_data).bodyPit1_1=ntohs((data_in->point_frame_data).bodyPit1_1);
    //(data_out->point_frame_data).bodyPit1_2=ntohs((data_in->point_frame_data).bodyPit1_2);//2023.9.11
    (data_out->point_frame_data).bodysize1_1=ntohs((data_in->point_frame_data).bodysize1_1); //2023.9.11

    (data_out->point_frame_data).used_time1=ntohs((data_in->point_frame_data).used_time1); //short

    (data_out->point_frame_data).bodyAzu2_0=ntohs((data_in->point_frame_data).bodyAzu2_0); //short
    (data_out->point_frame_data).bodyAzu2_1=ntohs((data_in->point_frame_data).bodyAzu2_1);
    //(data_out->point_frame_data).bodyAzu2_2=ntohs((data_in->point_frame_data).bodyAzu2_2);//2023.9.11
    (data_out->point_frame_data).bodysize2_0=ntohs((data_in->point_frame_data).bodysize2_0); //2023.9.11
    (data_out->point_frame_data).bodyPit2_0=ntohs((data_in->point_frame_data).bodyPit2_0); //short
    (data_out->point_frame_data).bodyPit2_1=ntohs((data_in->point_frame_data).bodyPit2_1);
    //(data_out->point_frame_data).bodyPit2_2=ntohs((data_in->point_frame_data).bodyPit2_2); //2023.9.11
    (data_out->point_frame_data).bodysize2_1=ntohs((data_in->point_frame_data).bodysize2_1); //2023.9.11
    (data_out->point_frame_data).used_time2=ntohs((data_in->point_frame_data).used_time2); //short

    (data_out->point_frame_data).bodyA=ntohl((data_in->point_frame_data).bodyA);
    (data_out->point_frame_data).bodyB=ntohl((data_in->point_frame_data).bodyB);
    (data_out->point_frame_data).bodyC=ntohl((data_in->point_frame_data).bodyC);
    (data_out->point_frame_data).bodyX=ntohl((data_in->point_frame_data).bodyX);
    (data_out->point_frame_data).bodyY=ntohl((data_in->point_frame_data).bodyY);
    (data_out->point_frame_data).bodyZ=ntohl((data_in->point_frame_data).bodyZ);

    (data_out->target_frame_data).bodyA=ntohl((data_in->target_frame_data).bodyA);
    (data_out->target_frame_data).bodyB=ntohl((data_in->target_frame_data).bodyB);
    (data_out->target_frame_data).bodyC=ntohl((data_in->target_frame_data).bodyC);
    (data_out->target_frame_data).bodyX=ntohl((data_in->target_frame_data).bodyX);
    (data_out->target_frame_data).bodyY=ntohl((data_in->target_frame_data).bodyY);
    (data_out->target_frame_data).bodyZ=ntohl((data_in->target_frame_data).bodyZ);
    //2023.12.27新加
    (data_out->target_frame_data).bodyAzu=ntohs((data_in->target_frame_data).bodyAzu);
    (data_out->target_frame_data).bodyPit=ntohs((data_in->target_frame_data).bodyPit);


    //2023.9.11





   // (data_out->target_frame_data).bodyA=ntohs((data_in->target_frame_data).bodyA); //int
    //(data_out->target_frame_data).bodyB=ntohs((data_in->target_frame_data).bodyB);  //int
    //(data_out->target_frame_data).bodyC=ntohs((data_in->target_frame_data).bodyC); //int
   // (data_out->target_frame_data).bodyX=ntohl((data_in->target_frame_data).bodyX); //int
   // (data_out->target_frame_data).bodyY=ntohl((data_in->target_frame_data).bodyY); //int
   // (data_out->target_frame_data).bodyZ=ntohl((data_in->target_frame_data).bodyZ); //int
    (data_out->target_frame_data).used_time=ntohs((data_in->target_frame_data).used_time); //short
   // (data_out->target_frame_data).ditance_data=ntohl((data_in->target_frame_data).ditance_data);

    //新添加的在轨的高低字节交换  2023.5.4
    data_out->zg_rightcount=ntohs(data_in->zg_rightcount);
    data_out->zg_wrongcount=ntohs(data_in->zg_wrongcount);
    data_out->bitmap_zg_rightcount=ntohs(data_in->bitmap_zg_rightcount);  //2023.11.3

    data_out->bitmap_lenthall=ntohs(data_in->bitmap_lenthall);


    #ifdef  USE_PRINTF_YC
    /*
    printf("data_in->point_frame_data.work_mode %x\n",data_in->work_mode);
    printf("data_in->point_frame_data.camer1_sum %x\n",data_in->point_frame_data.camera1_sum);
    printf("data_in->point_frame_data.used_time1 %d\n",data_in->point_frame_data.used_time1);
    printf("data_in->point_frame_data.used_time2 %d\n",data_in->point_frame_data.used_time2);
    printf("target_frame_data used_time %x\n",data_in->target_frame_data.used_time);
    printf("bodyA:%d bodyB:%d  bodyC:%d  bodyX:%d  bodyY:%d  bodyZ:%d\n", (data_in->target_frame_data).bodyA, (data_out->target_frame_data).bodyB,\
             (data_in->target_frame_data).bodyC, (data_in->target_frame_data).bodyX,\
          (data_in->target_frame_data).bodyY, (data_in->target_frame_data).bodyZ);
          */



    #endif

    //在此计算校验和



     data_out->sum=cmputCheckSum(((unsigned char*)data_out)+2,(sizeof(tmPackage)-3));



     //printf("point0 %x %x\n",(data_in->point_frame_data).bodyAzu1_0,(data_in->point_frame_data).bodyPit1_0);
     //printf("pointfar %x %x\n",(data_in->point_frame_data).farcamera_bodyAzu,(data_in->point_frame_data).farcamera_bodyPit);
}


long getSystime(unsigned char time[6])
{
    timeval tmp;
    gettimeofday(&tmp,NULL);

    long msec = tmp.tv_sec*1000+tmp.tv_usec/1000;
    time[0] = (msec&0xFF0000000000)>>40;
    time[1] = (msec&0xFF00000000)>>32;
    time[2] = (msec&0xFF000000)>>24;
    time[3] = (msec&0xFF0000)>>16;
    time[4] = (msec&0xFF00)>>8;
    time[5] = (msec&0xFF)>>0;
    return msec;
}


void SetCameraMode()
{
    int ret;
    ret=SendUartIns(fd[3],3);  //c监视
    if(ret==FALSE)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("发送c失败\n");
        #endif
    }
    ret=SendUartIns(fd[5],2);  //远场监测
    if(ret==FALSE)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("发送远场失败\n");
        #endif
    }
    //广域C&D&远视场&近视场监视
    ret=SendUartIns(fd[1],3);  //a监视
    if(ret==FALSE)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("发送a失败\n");
        #endif
    }
    ret=SendUartIns(fd[2],3);  //b监视
    if(ret==FALSE)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("发送b失败\n");
        #endif
    }
    ret=SendUartIns(fd[4],3);  //c监视
    if(ret==FALSE)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("发送c失败\n");
        #endif
    }
    ret=SendUartIns(fd[6],2);  //近场监测
    if(ret==FALSE)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("发送近场失败\n");
        #endif
    }
}

/***********************************************************
 * 1.遥控指令解析  (接收反熔丝串口过来的指令  )
 *
 * 包括遥控指令、注数指令、取遥测指令
 *
 * 发送给相机的指令
 *
 * 曝光开  cmd_send  0x01010101
 * 曝光关  cmd_send  0x02020202
 * 待机    cmd_send  0x03030303
 * 退出待机 cmd_send  0x04040404
 *
 * //注入和指令
 * ********************************************************/

void getrefreshSysTim(unsigned char time[])
{
    //unsigned int mCpuTickMs = 0;
    unsigned int totalsecond=0;
    unsigned int totalms=0;
    timeval tmp;
    gettimeofday(&tmp,NULL);
    totalsecond=(tmp.tv_sec-1483228800);
    totalms=(tmp.tv_usec/100);  //转成0.1ms
    time[0]=(totalsecond>>24);
    time[1]=(totalsecond>>16);
    time[2]=(totalsecond>>8);
    time[3]=(totalsecond);
    time[4]=(totalms>>8);
    time[5]=(totalms);

    //printf("time:%x,%x,%x,%x,%x,%x\n",time[0],time[1],time[2],time[3],\
          // time[4],time[5]);

}
#if 0
FILE *fd_yc;
unsigned flag_write=0;
#endif
unsigned int flag_yc_wrong=0;


void setcalibrationTime(unsigned char gncTime[])
{
    unsigned int sysHighTime = 0;
    unsigned int sysLowTime = 0;
    unsigned long time_real=0;
    unsigned long time_realus=0;

#if 0
//    gncTime[0]=0x10;
//    gncTime[1]=0x87;
//    gncTime[2]=0x13;
//    gncTime[3]=0xa4;
//    gncTime[4]=0x25;
//    gncTime[5]=0xb;


    gncTime[0]=0x10;
    gncTime[1]=0x87;
    gncTime[2]=0x13;
    gncTime[3]=0xac;
    gncTime[4]=0x7;
    gncTime[5]=0xb;
#endif


    sysHighTime =((gncTime[0] << 24)|(gncTime[1]<<16)|(gncTime[2]<<8)|(gncTime[3]));
    sysLowTime = ((gncTime[4] << 8 | gncTime[5]));  //0.1*1000=100us

   // printf("sysHighTime%x sysLowTime:%x\n",sysHighTime,sysLowTime);

    time_real=(sysHighTime +1483228800);  //1483228800
    time_realus=sysLowTime*100;

    timeval tmp;
    tmp.tv_sec=time_real;
    tmp.tv_usec=time_realus;

    //printf("sysHighTime%d sysLowTime:%d time_real:%d time_realus:%d\n",sysHighTime,sysLowTime,time_real,time_realus);
    if(settimeofday(&tmp,NULL)<0)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("settime fail\n");
        #endif
    }

}

#ifdef  TEST_DELAY
unsigned int delay_flag=0;
unsigned int delay_flag_main=0;
#endif


void TelctlProcess(unsigned char *data,int fd_out)
{
    //判断校验和
    unsigned short lenth=((data[3]<<8)|data[4]);
    unsigned short  lenth_yk;
    unsigned int i;
    unsigned int cmd;
    unsigned int j;
    //unsigned short cmd_send;

    unsigned int ins_lenth=0;
    unsigned int ins_sum=0;
    unsigned int ins_sum_before=0;
    unsigned short inject_data;

    #ifdef SEND_YC_YQ_ONLY
    unsigned  int tm_lenth;
    unsigned int tm_lenth_bitmap;
    #endif
    int ret;

    //int monitor_reset;  //增加monitor复位
    int image_reset; //增加image复位
    int resetFactory_ret;
    int zg_set_flag;

    #if 0
    static long lonCurrTimetarget;
    static long lonCurrTimetarget1;
    static long lonCurrTimetarget2;
    static long lonCurrTimetarget3;
    unsigned char currTime[6] = {0};
    #endif

     #ifdef  TEST_DELAY
     long lonCurrTimetarget_before;
     static long lonCurrTimetarget;
     unsigned char currTime[6] = {0};
     #endif

     //unsigned Save_Number;

      int InitGpio_flag[6];







    //存个文件看一下

    #if 0

    if(flag_write==0)
    {
       fd_yc = fopen("/mnt/ycsend.dat", "wb");
       flag_write=1;
     }
    #endif
    //指令类型码
    unsigned char ins_typecode=data[2];

    #if 0
    fwrite(data,lenth+6, 1, fd_yc);
    fflush(fd_yc);
    #endif

    flag_yc_wrong=0;
    if(checkSum(data+2,lenth+3,data[lenth+5])==SUCCESS)  //0xeb90的结构
    //if(1)
    {

        //遥控包的格式  4.4 前两字节固定 0x10 0x61 0x1e遥控指令类型码
        if(data[5]==0x10 && data[6]==0xc7 && ins_typecode==0x1e)//遥控包的头是对的
         {

            lenth_yk=((((data[9]<<8)|data[10])+1)-2);  //除去服务类型和服务子类型

            //根据服务类型判定是否是立即数/注数序列
            if(data[11]==0x50) //立即数/注数指令类型  p8表10
            {
                 //确认指令条数
                for(i=0;i<data[12];i++)
                {
                    ins_lenth=data[14+ins_sum];  //该条指令长度记录
                    ins_sum_before=ins_sum+ins_lenth;
                    if(ins_sum_before>lenth_yk)
                     {
                        #ifdef IF_PRINTF_MONITOR
                        printf("lenth wrong %d\n",ins_sum_before);
                        #endif
                        break;
                    }
                    //if(data[13+i*8]==0xf8)  //遥控指令类型码
                     if(data[13+ins_sum]==0xf8)  //遥控指令类型码
                     {
                        cmd=((data[17+i*8]<<24)|(data[18+i*8]<<16)|(data[19+i*8]<<8)|(data[20+i*8]));
                        tmdata.ins_count++;
                        #ifdef IF_PRINTF_MONITOR
                        printf("tmdata.ins_count:%d\n",tmdata.ins_count);
                        #endif

                        //不同指令码分发不同内容  放在不同缓存空间？？
                        switch(cmd)
                        {
                          //更改相机模式
                          case 0x01010101: //广域相机a自动曝光开
                            if(enProgramModify==0xff)
                            {


                               ret=SendUartIns(fd[1],0);

                               if(ret==FALSE)
                               {
                                   #ifdef IF_PRINTF_MONITOR
                                   printf("发送失败%x\n",cmd);
                                   #endif
                                   tmdata.wrong_ins_count++;
                               }


                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域相机a自动曝光开\n");
                               #endif
                             }
                            else
                            {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机a自动曝光未开，在轨模式\n");
                                #endif
                            }

                            break;
                          case 0x02020202: //广域相机a自动曝光关
                            if(enProgramModify==0xff)
                            {

                                data_exposure_flag[0]=0x00;
                                ret=SendUartIns(fd[1],1);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机a自动曝光关\n");
                                #endif
                             }
                            else
                            {
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域相机a自动曝光未关，在轨模式\n");
                               #endif
                            }

                            break;
                          case 0x03030303: //广域相机b自动曝光开
                            if(enProgramModify==0xff)
                            {

                               ret=SendUartIns(fd[2],0);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机b自动曝光开\n");
                                #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域相机b自动曝光未开，在轨模式\n");
                                 #endif
                            }

                            break;
                          case 0x04040404: //广域相机b自动曝光关
                            if(enProgramModify==0xff)
                            {

                                data_exposure_flag[1]=0x00;
                                ret=SendUartIns(fd[2],1);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机b自动曝光关\n");
                                #endif
                             }
                            else
                            {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机b自动曝光未关，在轨模式\n");
                                #endif
                            }

                            break;
                          case 0x05050505: //广域相机c自动曝光开
                            if(enProgramModify==0xff)
                            {


                                ret=SendUartIns(fd[3],0);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域相机c自动曝光开\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机c自动曝光未开，在轨模式\n");
                                #endif
                            }

                            break;
                          case 0x06060606: //广域相机c自动曝光关
                            if(enProgramModify==0xff)
                            {

                                data_exposure_flag[2]=0x00;
                                ret=SendUartIns(fd[3],1);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                     tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机c自动曝光关\n");
                                #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域相机c自动曝光未关，在轨模式\n");
                                 #endif
                            }

                            break;
                          case 0x07070707: //广域相机d自动曝光开
                            if(enProgramModify==0xff)
                            {


                                ret=SendUartIns(fd[4],0);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域相机d自动曝光开\n");
                               #endif
                             }
                            else
                            {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域相机d自动曝光未开，在轨模式\n");
                                 #endif
                            }

                            break;
                          case 0x08080808: //广域相机d自动曝光关
                            if(enProgramModify==0xff)
                            {

                                data_exposure_flag[3]=0x00;
                                ret=SendUartIns(fd[4],1);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域相机d自动曝光关\n");
                                #endif
                             }
                            else
                             {
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域相机d自动曝光未关，在轨模式\n");
                               #endif
                            }

                            break;
                          case 0x09090909: //远场相机自动曝光开
                            if(enProgramModify==0xff)
                            {

                                ret=SendUartIns(fd[5],0);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                              // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("远场相机自动曝光开\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("远场相机自动曝光未开，在轨模式\n");
                                #endif
                            }

                            break;
                          case 0x0a0a0a0a: //远场相机自动曝光关
                            if(enProgramModify==0xff)
                            {

                                data_exposure_flag[4]=0x00;
                                ret=SendUartIns(fd[5],1);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("远场相机自动曝光关\n");
                                #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("远场相机自动曝光未关，在轨模式\n");
                                 #endif
                            }

                            break;
                          case 0x0b0b0b0b: //近场相机自动曝光开
                            if(enProgramModify==0xff)
                            {

                                ret=SendUartIns(fd[6],0);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("近场相机自动曝光开\n");
                                #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("近场相机自动曝光未开，在轨模式\n");
                                #endif
                            }

                            break;
                          case 0x0c0c0c0c: //近场相机自动曝光关
                            if(enProgramModify==0xff)
                            {

                                data_exposure_flag[5]=0x00;
                                ret=SendUartIns(fd[6],1);
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送失败%x\n",cmd);
                                    #endif
                                    tmdata.wrong_ins_count++;
                                }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("近场相机自动曝光关\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("近场相机自动曝光未关，在轨模式\n");
                                #endif
                            }

                            break;
                          //模式切换
                          case 0x0d0d0d0d:  //0D广域A&B告警、广域C&D&远视场&近视场监视
                            if(enProgramModify==0xff)
                            {

                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[0][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);


                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[1],2);  //a告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[2],2);  //b告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[3],3);  //c监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],3);  //d监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],3);  //远场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif

                                #ifdef HandCom
                                //模式切换时全部关闭
                                for(j=4;j<10;j++)
                                {
                                   InitGpio_flag[j-4]=InitGpioout(0,j);
                                }


                                if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                                #endif







                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确

                               #ifdef IF_PRINTF_MONITOR
                               printf("广域A&B告警、广域C&D&远视场&近视场监视\n");
                               #endif

                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域A&B告警、广域C&D&远视场&近视场监视未开，在轨模式\n");
                                 #endif
                            }

                           break;
                          case 0x0e0e0e0e: //广域A&C告警、广域B&D&远视场&近视场监视
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[1][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确


                                #ifndef HandCom
                               //切相机 设置相机
                               ret=SendUartIns(fd[1],2);  //a告警
                               if(ret==FALSE)
                               {
                                   #ifdef IF_PRINTF_MONITOR
                                   printf("发送相机a告警失败%x\n",cmd);
                                   #endif
                               }
                               ret=SendUartIns(fd[3],2);  //c告警
                               if(ret==FALSE)
                               {
                                   #ifdef IF_PRINTF_MONITOR
                                   printf("发送相机c告警失败%x\n",cmd);
                                   #endif
                               }
                               //广域B&D&远视场&近视场监视
                               ret=SendUartIns(fd[2],3);  //b监视
                               if(ret==FALSE)
                               {
                                   #ifdef IF_PRINTF_MONITOR
                                   printf("发送相机b告警失败%x\n",cmd);
                                   #endif
                               }
                               ret=SendUartIns(fd[4],3);  //d监视
                               if(ret==FALSE)
                               {
                                   #ifdef IF_PRINTF_MONITOR
                                   printf("发送相机d告警失败%x\n",cmd);
                                   #endif
                               }
                               ret=SendUartIns(fd[5],3);  //远场监视
                               if(ret==FALSE)
                               {
                                   #ifdef IF_PRINTF_MONITOR
                                   printf("发送相机远场告警失败%x\n",cmd);
                                   #endif
                               }
                               ret=SendUartIns(fd[6],3);  //近场监视
                               if(ret==FALSE)
                               {
                                   #ifdef IF_PRINTF_MONITOR
                                   printf("发送相机近场监视失败%x\n",cmd);
                                   #endif
                               }
                               #endif


                                #ifdef HandCom
                                //模式切换时全部关闭
                                for(j=4;j<10;j++)
                                {
                                   InitGpio_flag[j-4]=InitGpioout(0,j);
                                }

                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }


                                #endif



                               #ifdef IF_PRINTF_MONITOR
                               printf("广域A&C告警、广域B&D&远视场&近视场监视\n");
                               #endif

                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域A&C告警、广域B&D&远视场&近视场监视未开，在轨模式\n");
                                 #endif
                            }

                            break;
                          case 0x0f0f0f0f:  //广域A&D告警、广域B&C&远视场&近视场监视
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[2][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);


                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[1],2);  //a告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],2);  //d告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[2],3);  //b监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[3],3);  //c监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],3);  //远场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef HandCom
                                //模式切换时全部关闭
                                for(j=4;j<10;j++)
                                {
                                   InitGpio_flag[j-4]=InitGpioout(0,j);
                                }

                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                                #endif

                               #ifdef IF_PRINTF_MONITOR
                               printf("广域A&D告警、广域B&C&远视场&近视场监视\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域A&D告警、广域B&C&远视场&近视场监视未开，在轨模式\n");
                                #endif
                            }


                            break;
                          case 0x10101010: //广域B&C告警、广域A&D&远视场&近视场监视
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[3][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);


                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[2],2);  //b告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[3],2);  //c告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[1],3);  //a监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],3);  //d监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],3);  //远场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif



                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }

                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确

                               #ifdef IF_PRINTF_MONITOR
                               printf("广域B&C告警、广域A&D&远视场&近视场监视\n");
                               #endif
                             }
                             else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域B&C告警、广域A&D&远视场&近视场监视未开，在轨模式\n");
                                #endif
                            }

                            break;
                          case 0x11111111:  //广域B&D告警、广域A&C&远视场&近视场监视
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[4][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);


                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[2],2);  //b告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],2);  //d告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[1],3);  //a监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[3],3);  //c监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],3);  //远场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif


                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif


                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                              #ifdef IF_PRINTF_MONITOR
                               printf("广域B&D告警、广域A&C&远视场&近视场监视\n");
                              #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                 printf("广域B&D告警、广域A&C&远视场&近视场监视未开，在轨模式\n");
                                #endif


                            }


                            break;
                          case 0x12121212:  //广域C&D告警、广域A&B&远视场&近视场监视
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[5][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);
                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确


                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[3],2);  //c告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],2);  //d告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[1],3);  //a监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[2],3);  //b监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],3);  //远场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif



                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                     InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                                #ifdef IF_PRINTF_MONITOR
                                printf("广域C&D告警、广域A&B&远视场&近视场监视\n");
                                #endif

                             }
                            else
                             {
                                  #ifdef IF_PRINTF_MONITOR
                                  printf("广域C&D告警、广域A&B&远视场&近视场监视未开，在轨模式\n");
                                  #endif
                            }


                            break;
                          case 0x13131313:    //远近视场协同测量、广域A&B&C&D监视

                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[6][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);


                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[5],2);  //远场
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送远场相机告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],2);  //近场
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送近场相机告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[1],3);  //a监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[2],3);  //b监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[3],3);  //c监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],3);  //d监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif



                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                     InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif


                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("远近视场协同测量、广域A&B&C&D监视\n");
                                #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("远近视场协同测量、广域A&B&C&D监视，在轨模式\n");
                                 #endif
                             }

                            break;
                          case 0x14141414:   //广域A&B&C&D轮询告警、远视场&近视场监视

                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=1;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=1;
                                memcpy(mode_if_changesend,&mode_listsend[0][0],6);
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);

                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[1],2);  //a告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[2],2);  //b告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }

                                ret=SendUartIns(fd[3],2);  //c告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],2);  //d告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],3);  //远场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif




                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                     InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif


                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                 printf("广域A&B&C&D轮询告警、远视场&近视场监视\n");
                                #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域A&B&C&D轮询告警、远视场&近视场监视，在轨模式\n");
                                #endif
                            }


                            break;

                        case 0x15151515:  //广域A待机
                            if(enProgramModify==0xff)
                            {

//                                ret=SendUartIns(fd[1],4);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[0]=0;
//                                    SetWorkmode(tmdata.work_mode);
//                                }

                                //2023.12.21
                                 InitGpio_flag[0]=InitGpioout(0,4);
                                 if(InitGpio_flag[0]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }

                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域a待机模式\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域a待机，在轨模式\n");
                                #endif
                            }

                            //mode_if_free[0]=0;
                             break;
                        case 0x16161616:  //广域B待机
                            if(enProgramModify==0xff)
                            {

//                                ret=SendUartIns(fd[2],4);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[1]=0;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(0,5);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域b待机模式\n");
                               #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域b待机，在轨模式\n");
                                 #endif
                            }

                           // mode_if_free[1]=0;

                             break;
                        case 0x17171717:  //广域C待机
                            if(enProgramModify==0xff)
                            {

//                                ret=SendUartIns(fd[3],4);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[2]=0;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(0,6);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域c待机\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域c待机，在轨模式\n");
                                #endif
                            }
                           // mode_if_free[2]=0;

                             break;
                        case 0x18181818:  //广域d待机
                            if(enProgramModify==0xff)
                            {

//                                ret=SendUartIns(fd[4],4);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[3]=0;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(0,7);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域d待机\n");
                                #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域d待机，在轨模式\n");
                                #endif
                            }
                            //mode_if_free[0]=3;

                             break;
                        case 0x19191919:  //远场待机
                            if(enProgramModify==0xff)
                            {

//                                ret=SendUartIns(fd[5],4);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[4]=0;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(0,8);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("远场待机\n");
                                #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("远场待机，在轨模式\n");
                                 #endif
                            }
                            //mode_if_free[4]=0;

                             break;
                        case 0x1a1a1a1a:  //近场待机
                            if(enProgramModify==0xff)
                            {

//                                ret=SendUartIns(fd[6],4);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[5]=0;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(0,9);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("近场待机\n");
                                #endif
                             }
                            else
                            {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("近场待机，在轨模式\n");
                                 #endif
                            }
                            //mode_if_free[5]=0;

                             break;
                        case 0x1b1b1b1b:  //广域a退出待机
                            if(enProgramModify==0xff)
                            {

                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(1,4);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }


//                                ret=SendUartIns(fd[1],5);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                   #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[0]=1;
//                                    SetWorkmode(tmdata.work_mode);
//                                }





                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域a退出待机\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域a退出待机，在轨模式\n");
                                #endif
                            }
                            //mode_if_free[0]=1;

                             break;
                        case 0x1c1c1c1c:  //广域b退出待机
                            if(enProgramModify==0xff)
                            {

                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(1,5);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }


//                                ret=SendUartIns(fd[2],5);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[1]=1;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域b退出待机\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域b退出待机，在轨模式\n");
                                #endif
                            }
                            //mode_if_free[1]=1;

                             break;
                        case 0x1d1d1d1d:  //广域c退出待机
                            if(enProgramModify==0xff)
                            {

                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(1,6);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }


//                                ret=SendUartIns(fd[3],5);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[2]=1;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                              // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域c退出待机\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域c退出待机，在轨模式\n");
                                #endif
                            }
                            //mode_if_free[2]=1;

                             break;
                        case 0x1e1e1e1e:  //广域d退出待机
                            if(enProgramModify==0xff)
                            {

                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(1,7);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }


//                                ret=SendUartIns(fd[4],5);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[3]=1;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("广域d退出待机\n");
                               #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域d退出待机，在轨模式\n");
                                #endif
                            }
                            //mode_if_free[3]=1;

                             break;
                        case 0x1f1f1f1f:  //远场退出待机
                            if(enProgramModify==0xff)
                            {

                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(1,8);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }
                               // sleep(2);


//                                ret=SendUartIns(fd[5],5);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[4]=1;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                               #ifdef IF_PRINTF_MONITOR
                               printf("远场退出待机\n");
                               #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("远场退出待机，在轨模式\n");
                                 #endif

                            }
                            //mode_if_free[4]=1;

                             break;
                        case 0x20202020:  //近场退出待机
                            if(enProgramModify==0xff)
                            {

                                //2023.12.21
                                InitGpio_flag[0]=InitGpioout(1,9);
                                if(InitGpio_flag[0]==FALSE)
                                {

                                    tmdata.wrong_ins_count++;
                                }

//                                ret=SendUartIns(fd[6],5);
//                                if(ret==FALSE)
//                                {
//                                    #ifdef IF_PRINTF_MONITOR
//                                    printf("发送失败%x\n",cmd);
//                                    #endif
//                                }
//                                else
//                                 {
//                                    mode_if_free[5]=1;
//                                    SetWorkmode(tmdata.work_mode);
//                                }
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("近场退出待机\n");
                                #endif
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("近场退出待机，在轨模式\n");
                                 #endif
                            }
                            //mode_if_free[5]=1;

                             break;
                         case 0x21212121:  //20M时钟
                            if(enProgramModify==0xff)
                            {


                               SetClockFre(0);
                               clock_status=0;
                               #ifdef IF_PRINTF_MONITOR
                               printf("20M时钟设置完成\n");
                               #endif
                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                             }
                            else
                            {
                                #ifdef IF_PRINTF_MONITOR
                                printf("20M时钟未设置成功，在轨模式\n");
                                #endif
                            }

                             break;
                        case 0x22222222:  //10M时钟
                           if(enProgramModify==0xff)
                           {

                               SetClockFre(1);
                               clock_status=1;
                               #ifdef IF_PRINTF_MONITOR
                               printf("10M时钟设置完成\n");
                               #endif

                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                           else
                            {
                               #ifdef IF_PRINTF_MONITOR
                               printf("10M时钟未设置成功，在轨模式\n");
                               #endif
                           }
                            break;

                          case 0x23232323:  //自检模式  //系统里保存的图
                            if(enProgramModify==0xff)
                            {
                                //不仅相机进入自检并且取系统图进算法
                                //控制相机进入自检
                                //cmd_send=0x0505;

                                //相机6路进入自检模式
                                /*
                                for(int i=1;i<7;i++)
                                {
                                    ret=SendUartIns(fd[i],send_uart_camera,(unsigned char*)&cmd_send,2,0x22);
                                    if(ret==FALSE)
                                    {
                                        printf("发送失败%x\n",cmd);
                                    }
                                }
                                */
                                work_mode_tj=2;

                               // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确

                                #ifdef IF_PRINTF_MONITOR
                                printf("设置6路相机进入自检\n");
                                #endif


                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功  自检模式%x\n",cmd);
                                #endif

                              }
                            else
                            {
                                #ifdef IF_PRINTF_MONITOR
                                printf("未自检模式，在轨模式\n");
                                #endif
                            }

                            break;
                          case 0x24242424:  //测试模式
                            if(enProgramModify==0xff)
                            {
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功  测试模式%x\n",cmd);
                                #endif
                                work_mode_tj=1;
                               // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("未测试模式，在轨模式\n");
                                 #endif
                            }
                            break;
                          case 0x25252525: //工作模式  //进行存储的图像的自检

                            if(enProgramModify==0xff)
                            {
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功  工作模式%x\n",cmd);
                                #endif
                                work_mode_tj=0;
                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("未工作模式，在轨模式\n");
                                 #endif
                            }

                            break;
                          case  0x26262626://在轨编程模式
                            zg_status=1;
                            initOnlineProgram(); //初始化在轨编程  log都设置了标志位 ptr_log->flag_online
                            setLog(LOG_ROUTE, sizeof(LOG_ROUTE));
                            enProgramModify=0xaa;  //保护指令状态
                            //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            #ifdef IF_PRINTF_MONITOR
                            printf("在轨编程模式\n");
                            #endif
                            break;
                         case 0x27272727: // 在轨编程完整实施                            
                             zg_set_flag=executeOnlineProgram(); //初始化在轨编程  log都设置了标志位 ptr_log->flag_online
                             if(zg_set_flag==0)
                             {
                                 //zg_status=2;
                                 zg_excut=1;
                                 log_data.flag_update=0xff;
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("在轨编程完整实施成功\n");
                                 #endif

                             }
                             else
                             {
                                  zg_excut=0;
                                  #ifdef IF_PRINTF_MONITOR
                                  printf("在轨编程完整实施失败\n");
                                  #endif
                                  tmdata.wrong_ins_count++;
                             }
                             setLog(LOG_ROUTE, sizeof(LOG_ROUTE));
                             enProgramModify=0xaa;  //保护指令状态
                             //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                             #ifdef IF_PRINTF_MONITOR

                             #endif
                             break;
                         case 0x28282828:
                           // exitOnlineProgram(); //退出在轨编程模式  log都设置了标志位 ptr_log->flag_online
                            //log_data.zg_rightcount=0x5;
                            //log_data.zg_wrongcount=0x8;

                            log_data.flag_online=0x55;

                            //log_data.zg_rightcount=0x0;
                            //log_data.zg_wrongcount=0x0;
                            //计数也可以归零了

                            setLog(LOG_ROUTE, sizeof(LOG_ROUTE));
                            enProgramModify=0xff;
                            zg_status=0;
                            //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            #ifdef IF_PRINTF_MONITOR
                             printf("退出在轨编程模式\n");
                            #endif
                            break;
                          case 0x29292929: //准备断电指令
                            //if(enProgramModify==0xff)
                            //{
                                setLog(LOG_ROUTE, sizeof(LOG_ROUTE));
                                flag_power_off = 1;
                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("准备断电指令\n");
                                #endif
                             //}
                            //else
                            // {
                                // printf("准备断电指令未执行\n");
                            //}
                            break;
                          case 0x2a2a2a2a: //数传开

                            if(enProgramModify==0xff)
                            {
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                #ifdef  IF_PRINTF_MONITOR
                                printf("网络发送成功  数传开%x\n",cmd);
                                #endif
                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                sc_on_off=1;
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                printf("数传未开，未执行\n",cmd);
                                #endif
                            }
                            break;
                          case 0x2b2b2b2b: //数传关
                            #if 0
                            char result[2048];
                            exectProgrammd5test("/mnt/spiflashtxt/image_process","/mnt/spiflashtxt/imageprocess.txt.md5");
                            #endif
                            if(enProgramModify==0xff)
                            {
                                sc_on_off=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功  数传关%x\n",cmd);
                                #endif
                               // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                             }
                             break;
                          case 0x2c2c2c2c: //恢复出厂设置

                            //if(enProgramModify==0xff)
                            {
                                 //resetFactory_flag=1;
                                 resetFactory_ret=resetFactory();  //后面要加上
                                 //测试用
                                 //resetFactory_ret=resetFactorytest();
                                 if(resetFactory_ret==0)
                                 {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("恢复出厂设置成功\n");
                                    #endif
                                    tmdata.resetfactory_count++;
                                 }
                                 else
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("恢复出厂设置失败\n");
                                     #endif
                                     tmdata.wrong_ins_count++;
                                 }
                                 //resetFactory_flag=0;


                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                            break;
                          case 0x2d2d2d2d: // 软复位
                            //monitor_reset=resetProcess("monitor");
                            //if(enProgramModify==0xff)
                            {
                                //image_reset=resetProcess("image_process");
                                image_reset=system("bash  /usr/bin/kill_image.sh");
                                #ifdef IF_PRINTF_MONITOR
                                printf("软复位成功 %d\n",image_reset);
                                #endif
                                if( image_reset!=1 && image_reset!=127)
                                 {
                                    tmdata.software_resetcount++;
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("软复位成功\n");
                                    #endif
                                }
                                else
                                 {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("软复位失败\n");
                                    #endif
                                    tmdata.wrong_ins_count++;

                                }
                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                              }
                            break;
                         // case 0x2e2e2e2e: // 硬复位
                            //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                          // break;
                         // case 0x2f2f2f2f: //取第1块bitmap
                            //processBitMapCmd(cmd);
                            //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            //break;

                        case 0x2f2f2f2f: // 广域A告警&远场测量，广域B&C&D&近视场监视  //2023.9.11
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[7][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);

                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[1],2);  //a告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[2],3);  //b监控
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[3],3);  //c监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],3);  //d监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],2);  //远场告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif


                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确


                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                               #ifdef IF_PRINTF_MONITOR
                               printf("广域A告警&远场测量，广域B&C&D&近视场监视 \n");
                               #endif

                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域A告警&远场测量，广域B&C&D&近视场监视未开在轨模式\n");
                                 #endif
                            }

                         break;
                       case 0x30303030:   //广域B告警&远场测量，广域A&C&D&近视场监视 //2023.9.11
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[8][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);

                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[1],3);  //a监控
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[2],2);  //b告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[3],3);  //c监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],3);  //d监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],2);  //远场告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],3);  //近场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif

                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确

                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                               #ifdef IF_PRINTF_MONITOR
                               printf("广域B告警&远场测量，广域A&C&D&近视场监视\n");
                               #endif

                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域B告警&远场测量，广域A&C&D&近视场监视未开，在轨模式\n");
                                 #endif
                            }
                            break;
                        case 0x31313131:   //广域C告警&远场测量，广域A&B&D&近视场监视  //2023.9.11
                             if(enProgramModify==0xff)
                             {
                                 roll_flag_mode=0;
                                 //发送给imageprocess软件
                                 SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                 udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                 mode_lx=0;
                                 memcpy(mode_if_changesend,&mode_listsend[9][0],6); //状态保存起来
                                 ModifyFjyk();
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("网络发送成功%x\n",cmd);
                                 #endif
                                 SetWorkmode(tmdata.work_mode);

                                 #ifndef HandCom
                                 //切相机  设置相机
                                 ret=SendUartIns(fd[1],3);  //a监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机a告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[2],3);  //b监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机b告警失败%x\n",cmd);
                                     #endif
                                 }
                                 //广域C
                                 ret=SendUartIns(fd[3],2);  //c告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机c告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[4],3);  //d监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机d告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[5],2);  //远场告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机远场告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[6],3);  //近场监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机近场监视失败%x\n",cmd);
                                     #endif
                                 }
                                 #endif

                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                                 //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确

                                #ifdef IF_PRINTF_MONITOR
                                printf("广域C告警&远场测量，广域A&C&D&近视场监视\n");
                                #endif

                              }
                             else
                              {
                                  #ifdef IF_PRINTF_MONITOR
                                  printf("广域C告警&远场测量，广域A&B&D&近视场监视未开，在轨模式\n");
                                  #endif
                             }
                             break;
                        case 0x32323232:   //广域D告警&远场测量，广域A&B&C&近视场监视  //2023.9.11
                             if(enProgramModify==0xff)
                             {
                                 roll_flag_mode=0;
                                 //发送给imageprocess软件
                                 SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                 udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                 mode_lx=0;
                                 memcpy(mode_if_changesend,&mode_listsend[10][0],6); //状态保存起来
                                 ModifyFjyk();
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("网络发送成功%x\n",cmd);
                                 #endif
                                 SetWorkmode(tmdata.work_mode);

                                 #ifndef HandCom
                                 //切相机  设置相机
                                 ret=SendUartIns(fd[1],3);  //a监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机a告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[2],3);  //b监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机b告警失败%x\n",cmd);
                                     #endif
                                 }
                                 //广域C
                                 ret=SendUartIns(fd[3],3);  //c监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机c告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[4],2);  //d告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机d告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[5],2);  //远场告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机远场告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[6],3);  //近场监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机近场监视失败%x\n",cmd);
                                     #endif
                                 }
                                 #endif


                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                                 //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef IF_PRINTF_MONITOR
                                printf("广域D告警&远场测量，广域A&B&C&近视场监视\n");
                               #endif

                              }
                             else
                              {
                                 #ifdef IF_PRINTF_MONITOR
                                  printf("广域D告警&远场测量，广域A&B&C&近视场监视未开，在轨模式\n");
                                 #endif
                             }
                             break;

                        case 0x33333333: // 广域A告警&近场测量，广域B&C&D&远视场监视  //2023.9.11
                            if(enProgramModify==0xff)
                            {
                                roll_flag_mode=0;
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                mode_lx=0;
                                memcpy(mode_if_changesend,&mode_listsend[11][0],6); //状态保存起来
                                ModifyFjyk();
                                #ifdef IF_PRINTF_MONITOR
                                printf("网络发送成功%x\n",cmd);
                                #endif
                                SetWorkmode(tmdata.work_mode);


                                #ifndef HandCom
                                //切相机  设置相机
                                ret=SendUartIns(fd[1],2);  //a告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机a告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[2],3);  //b监控
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机b告警失败%x\n",cmd);
                                    #endif
                                }
                                //广域C&D&远视场&近视场监视
                                ret=SendUartIns(fd[3],3);  //c监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机c告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[4],3);  //d监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机d告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[5],3);  //远场监视
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机远场告警失败%x\n",cmd);
                                    #endif
                                }
                                ret=SendUartIns(fd[6],2);  //近场告警
                                if(ret==FALSE)
                                {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("发送相机近场监视失败%x\n",cmd);
                                    #endif
                                }
                                #endif


                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确

                               #ifdef IF_PRINTF_MONITOR
                               printf("广域A告警&近场测量，广域B&C&D&远视场监视 \n");
                               #endif

                             }
                            else
                             {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域A告警&近场测量，广域B&C&D&远视场监视未开在轨模式\n");
                                 #endif
                            }

                         break;
                        case 0x34343434:   //广域B告警&近场测量，广域A&C&D&远视场监视 //2023.9.11
                             if(enProgramModify==0xff)
                             {
                                 roll_flag_mode=0;
                                 //发送给imageprocess软件
                                 SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                 udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                 mode_lx=0;
                                 memcpy(mode_if_changesend,&mode_listsend[12][0],6); //状态保存起来
                                 ModifyFjyk();
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("网络发送成功%x\n",cmd);
                                 #endif
                                 SetWorkmode(tmdata.work_mode);

                                 #ifndef HandCom
                                 //切相机  设置相机
                                 ret=SendUartIns(fd[1],3);  //a监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机a告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[2],2);  //b告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机b告警失败%x\n",cmd);
                                     #endif
                                 }
                                 //广域C&D&远视场&近视场监视
                                 ret=SendUartIns(fd[3],3);  //c监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机c告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[4],3);  //d监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机d告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[5],3);  //远场监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机远场告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[6],2);  //近场告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机近场监视失败%x\n",cmd);
                                     #endif
                                 }
                                 #endif

                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                                #ifdef IF_PRINTF_MONITOR
                                printf("广域B告警&近场测量，广域A&C&D&远场监视\n");
                                #endif

                              }
                             else
                              {
                                  #ifdef IF_PRINTF_MONITOR
                                  printf("广域B告警&近场测量，广域A&C&D&远视场监视未开，在轨模式\n");
                                  #endif
                             }
                             break;
                        case 0x35353535:   //广域C告警&近场测量，广域A&B&D&远视场监视  //2023.9.11
                             if(enProgramModify==0xff)
                             {
                                 roll_flag_mode=0;
                                 //发送给imageprocess软件
                                 SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                 udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                 mode_lx=0;
                                 memcpy(mode_if_changesend,&mode_listsend[13][0],6); //状态保存起来
                                 ModifyFjyk();
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("网络发送成功%x\n",cmd);
                                 #endif
                                 SetWorkmode(tmdata.work_mode);

                                 #ifndef HandCom
                                 //切相机  设置相机
                                 ret=SendUartIns(fd[1],3);  //a监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机a告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[2],3);  //b监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机b告警失败%x\n",cmd);
                                     #endif
                                 }
                                 //广域C
                                 ret=SendUartIns(fd[3],2);  //c告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机c告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[4],3);  //d监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机d告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[5],3);  //远场监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机远场告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[6],2);  //近场告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机近场监视失败%x\n",cmd);
                                     #endif
                                 }
                                 #endif

                                //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                                 #ifdef IF_PRINTF_MONITOR
                                printf("广域C告警&近场测量，广域A&C&D&远视场监视\n");
                                #endif

                              }
                             else
                              {
                                  #ifdef IF_PRINTF_MONITOR
                                  printf("广域C告警&近场测量，广域A&B&D&远视场监视未开，在轨模式\n");
                                 #endif
                             }
                             break;

                        case 0x36363636:   //广域D告警&近场测量，广域A&B&C&远视场监视  //2023.9.11
                             if(enProgramModify==0xff)
                             {
                                 roll_flag_mode=0;
                                 //发送给imageprocess软件
                                 SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                 udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                                 mode_lx=0;
                                 memcpy(mode_if_changesend,&mode_listsend[14][0],6); //状态保存起来
                                 ModifyFjyk();
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("网络发送成功%x\n",cmd);
                                 #endif
                                 SetWorkmode(tmdata.work_mode);

                                 #ifndef HandCom
                                 //切相机  设置相机
                                 ret=SendUartIns(fd[1],3);  //a监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机a告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[2],3);  //b监控
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机b告警失败%x\n",cmd);
                                      #endif
                                 }
                                 //广域C
                                 ret=SendUartIns(fd[3],3);  //c监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机c告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[4],2);  //d告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机d告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[5],3);  //远场监视
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机远场告警失败%x\n",cmd);
                                     #endif
                                 }
                                 ret=SendUartIns(fd[6],2);  //近场告警
                                 if(ret==FALSE)
                                 {
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("发送相机近场监视失败%x\n",cmd);
                                     #endif
                                 }
                                 #endif

                                 //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确

                                #ifdef HandCom
                                 //模式切换时全部关闭
                                 for(j=4;j<10;j++)
                                 {
                                    InitGpio_flag[j-4]=InitGpioout(0,j);
                                 }
                                 if(InitGpio_flag[0]==FALSE || InitGpio_flag[1]==FALSE||InitGpio_flag[2]==FALSE ||InitGpio_flag[3]==FALSE||InitGpio_flag[4]==FALSE ||InitGpio_flag[5]==FALSE)
                                 {

                                     tmdata.wrong_ins_count++;
                                 }
                                 #endif

                                #ifdef IF_PRINTF_MONITOR
                                printf("广域D告警&近场测量，广域A&B&C&远视场监视\n");
                                #endif

                              }
                             else
                              {
                                 #ifdef IF_PRINTF_MONITOR
                                 printf("广域D告警&近场测量，广域A&B&C&远视场监视未开，在轨模式\n");
                                 #endif
                             }
                             break;

                        case  0x37373737: // 远场进靶标

                            if(enProgramModify==0xff)
                            {
                                //发送给imageprocess软件
                                SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                                udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);


                                #ifdef IF_PRINTF_MONITOR
                                printf("远场进入靶标测量\n");
                                #endif
                             }
                            else
                             {
                                #ifdef IF_PRINTF_MONITOR
                                 printf("远场未进入靶标测量\n");
                                #endif
                             }

                            break;

                         case 0x38383838://恢复出厂设置1

                            //if(enProgramModify==0xff)
                            {
                                 //resetFactory_flag=1;
                                 resetFactory_ret=resetFactory1();  //后面要加上
                                 //测试用
                                 //resetFactory_ret=resetFactorytest();
                                 if(resetFactory_ret==0)
                                 {
                                    #ifdef IF_PRINTF_MONITOR
                                    printf("恢复出厂设置成功\n");
                                    #endif
                                    tmdata.resetfactory_count++;
                                 }
                                 else
                                 {
                                     tmdata.wrong_ins_count++;
                                     #ifdef IF_PRINTF_MONITOR
                                     printf("恢复出厂设置失败\n");
                                     #endif
                                 }
                                 //resetFactory_flag=0;


                               //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                            break;
                         case 0x39393939://开启广域A局部曝光
                            if(enProgramModify==0xff)
                            {
                               data_exposure_flag[0]=0x01;
                               #ifdef IF_PRINTF_MONITOR
                                printf("开启广域A局部曝光\n");
                               #endif
                             }
                            break;
                         case 0x3A3A3A3A://开启广域B局部曝光
                            if(enProgramModify==0xff)
                            {
                               data_exposure_flag[1]=0x01;
                               #ifdef IF_PRINTF_MONITOR
                                printf("开启广域B局部曝光\n");
                               #endif
                             }
                           break;
                         case 0x3B3B3B3B://开启广域C局部曝光
                            if(enProgramModify==0xff)
                            {
                               data_exposure_flag[2]=0x01;
                               #ifdef IF_PRINTF_MONITOR
                                printf("开启广域C局部曝光\n");
                               #endif
                             }
                          break;
                        case 0x3C3C3C3C://开启广域D局部曝光
                            if(enProgramModify==0xff)
                            {
                               data_exposure_flag[3]=0x01;
                               #ifdef IF_PRINTF_MONITOR
                                printf("开启广域D局部曝光\n");
                               #endif
                             }
                         break;
                        case 0x3D3D3D3D://开启远场曝光
                            if(enProgramModify==0xff)
                            {
                               data_exposure_flag[4]=0x01;
                               #ifdef IF_PRINTF_MONITOR
                                printf("开启远场曝光\n");
                               #endif
                             }
                         break;
                        case 0x3E3E3E3E://开启近场曝光
                            if(enProgramModify==0xff)
                            {
                               data_exposure_flag[5]=0x01;
                               #ifdef IF_PRINTF_MONITOR
                                printf("开启近场曝光\n");
                               #endif
                             }
                         break;

                         case 0x52525252://存图


                            //发送给imageprocess软件
                            #ifdef  TEST_DELAY
                             delay_flag=0;
                             delay_flag_main=0;
                             delay_flag_udp=0;
                             SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                             udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                             printf("测延迟\n");
                            #endif



                             #ifdef SAVE_PIC
                              SendImageSoftWare(send_net,(unsigned char*)&cmd,2,0x11);
                              udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
                              #ifdef IF_PRINTF_MONITOR
                              printf("存图\n");
                              #endif
                             #endif



                            break;


                         default:
                            if(enProgramModify==0xff)
                            {
                              //telctl_feedback(fd_out,0xff,0xe1); //数据传输正确
                              tmdata.wrong_ins_count++;
                            }
                            flag_yc_wrong=1;  //遇到了没有的指令
                            break;
                        }  //switch(cmd)
                } //if( data[13+ins_sum]==0xf8)  //遥控指令类型码
                else if(data[13+ins_sum]==0xf9 &&  enProgramModify==0xff) //注数指令码
                {
                       inject_data=((data[17+ins_sum]<<8)|data[18+ins_sum]);
                       tmdata.framesz_count++;


                       if(inject_data==0xaa01) //广域a相机曝光时间
                       {
                           if(enProgramModify==0xff)
                           {

                              cmd_send[0]=data[19+ins_sum];
                              cmd_send[1]=data[20+ins_sum];
                              cmd_send[2]=data[21+ins_sum];
                              cmd_send[3]=data[22+ins_sum];
                              SendUartInszs(fd[1],send_uart_camera, cmd_send,4);

                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else if(inject_data==0xaa02) //广域b相机曝光时间
                       {
                           if(enProgramModify==0xff)
                           {

                               cmd_send[0]=data[19+ins_sum];
                               cmd_send[1]=data[20+ins_sum];
                               cmd_send[2]=data[21+ins_sum];
                               cmd_send[3]=data[22+ins_sum];
                               SendUartInszs(fd[2],send_uart_camera, cmd_send,4);
                              // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else if(inject_data==0xaa03) //广域c相机曝光时间
                       {
                           if(enProgramModify==0xff)
                           {

                               cmd_send[0]=data[19+ins_sum];
                               cmd_send[1]=data[20+ins_sum];
                               cmd_send[2]=data[21+ins_sum];
                               cmd_send[3]=data[22+ins_sum];
                               SendUartInszs(fd[3],send_uart_camera, cmd_send,4);

                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }

                       }
                       else if(inject_data==0xaa04) //广域d相机曝光时间
                       {
                           if(enProgramModify==0xff)
                           {

                             cmd_send[0]=data[19+ins_sum];
                             cmd_send[1]=data[20+ins_sum];
                             cmd_send[2]=data[21+ins_sum];
                             cmd_send[3]=data[22+ins_sum];
                             SendUartInszs(fd[4],send_uart_camera, cmd_send,4);
                             //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else if(inject_data==0xaa05) //近场相机曝光时间
                       {
                           if(enProgramModify==0xff)
                           {

                               cmd_send[0]=data[19+ins_sum];
                               cmd_send[1]=data[20+ins_sum];
                               cmd_send[2]=data[21+ins_sum];
                               cmd_send[3]=data[22+ins_sum];
                               SendUartInszs(fd[5],send_uart_camera, cmd_send,4);
                             // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }

                       }
                       else if(inject_data==0xaa06) //远场相机曝光时间
                       {
                           if(enProgramModify==0xff)
                           {

                              cmd_send[0]=data[19+ins_sum];
                              cmd_send[1]=data[20+ins_sum];
                              cmd_send[2]=data[21+ins_sum];
                              cmd_send[3]=data[22+ins_sum];
                              SendUartInszs(fd[6],send_uart_camera, cmd_send,4);
                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else if(inject_data==0xaa07) //广域告警参数设置
                       {
                           if(enProgramModify==0xff)
                           {

                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else if(inject_data==0xaa08) //远场告警参数设置
                       {
                           if(enProgramModify==0xff)
                           {

                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else if(inject_data==0xaa09) //远场面目标参数设置
                       {
                           if(enProgramModify==0xff)
                           {

                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else if(inject_data==0xaa0a) //近场合作靶标测量设置
                       {
                           if(enProgramModify==0xff)
                           {

                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                            }
                       }
                       else
                       {
                           if(enProgramModify==0xff)
                           {
                             //telctl_feedback(fd_out,0xff,0xe1); //数据传输正确
                             //tmdata.wrong_ins_count++;
                             tmdata.wrong_framesz_count++;
                           }
                            flag_yc_wrong=1;  //遇到了没有的指令
                       }
                 }  //注数指令
                 else if(data[13+ins_sum]==0xa0) //准备文件上注指令
                 {
                         tmdata.ins_count++;
                        // 准备文件上注指令
                         if(data[16+ins_sum]==0x00)
                         {
                             if(enProgramModify==0xaa)
                             {


                              }
                             // telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                         }
                         else if(data[16+ins_sum]==0xcc) //启动校验指令
                         {
                             if(enProgramModify==0xff)
                             {


                              }
                             //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                         }
                         else if(data[16+ins_sum]==0xdd) //启动bitmap传输
                         {

                             bitmap_send_count=SendBitMapYcCount();
                             if(bitmap_send_count>0)
                              {
                                  bitmap_send_countck=0;
                                  bitmap_setcount=0;

                                  bitmap_send_flag=1;
                                  bitmap_success=0;
                                  tmdata.bitmap_lenthall=bitmap_lenth;

                              }
                             else
                             {
                                 bitmap_success=1;
                             }
                              //telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
                         }
                         else
                         {
                             //if(enProgramModify==0xaa)
                             {

                               tmdata.wrong_ins_count++;
                             }
                              flag_yc_wrong=1;  //遇到了没有的指令
                             //telctl_feedback(fd_out,0xff,0xe1); //数据传输错误
                         }
                  }
                 ins_sum+=ins_lenth;  //总共偏移了多少字节

             }  //for(i=0;i<data[12];i++)

             //看是否进入了default的分支，进入了，那就证明有错

         } //if(data[11]==0x50) //立即数/注数指令类型

          //错误遥控指令
         #ifdef SEND_YC_YQ

         if( flag_yc_wrong==1)
          {

              telctl_feedback(fd_out,0xff,0xe1); //数据传输错误
         }
         else
          {
             telctl_feedback(fd_out,0xaa,0xe1); //数据传输正确
         }

         #endif

    } //if(data[5]==0x10 && data[6]==0x61 && ins_typecode==0x1e)//遥控包的头是对的
     else if(data[5]==0x10 && data[6]==0xcf && ins_typecode==0x1e)  //应用程序标识符和指令类型码确认下？？？
     {

            if(log_data.flag_online==0xaa)
            {
                processReceivedPackOut(data+5,lenth-6); //处理接收到的小包，第5项开始为有效数据
            }
            else
            {
                #ifdef  IF_PRINTF_MONITOR
                printf("未在在轨编程模式\n");
                #endif
            }

            #if 0
            lonCurrTimetarget = getSystime(currTime);
            printf("lonCurrZS:%f\n",(lonCurrTimetarget-lonCurrTimetarget1)/1000.);
            lonCurrTimetarget1 =  lonCurrTimetarget;
            #endif


    }
    else if(ins_typecode==0x0f)  //常规遥测数据
    {

       #ifdef SEND_YC_YQ_ONLY
        //发送遥测
        pthread_mutex_lock(&mutex);

      if(bitmap_send_flag==0)
       {


        tm_lenth=SendTmData(&tmdata);

        //
        tmdata.target_frame_data.Alarm_staus=0xaa;
        //getrefreshSysTim(tmdata.target_frame_data.timesamp);



        ZhData(&tmdata,&tmdata_out);
        UART0_Send(fd_out,(char*)(&tmdata_out),tm_lenth);

        //
       #ifdef  TEST_DELAY
        if(tmdata.target_frame_data.ditance_data[1]==0x73  &&tmdata.target_frame_data.ditance_data[2]==0xd7&&delay_flag==0)
         {
               lonCurrTimetarget = getSystime(currTime);
               delay_flag=1;
               lonCurrTimetarget_before=(tmdata.target_frame_data.timesamp[0]|(tmdata.target_frame_data.timesamp[1]<<8)|(tmdata.target_frame_data.timesamp[2]<<16)|(tmdata.target_frame_data.timesamp[3]<<24)|(tmdata.target_frame_data.timesamp[4]<<32)|(tmdata.target_frame_data.timesamp[5]<<40));
               //printf("tmdata.target_frame_data:%x %x %x %x",tmdata.target_frame_data.timesamp[0],tmdata.target_frame_data.timesamp[1],tmdata.target_frame_data.timesamp[2],tmdata.target_frame_data.timesamp[3]);
               printf("采图时间：%d %d %d\n",lonCurrTimetarget,lonCurrTimetarget_before,(lonCurrTimetarget-lonCurrTimetarget_before));
          }
        #endif






        #if 0
        lonCurrTimetarget = getSystime(currTime);
        printf("lonCurryc:%f\n",(lonCurrTimetarget-lonCurrTimetarget1)/1000.);
        lonCurrTimetarget1 =  lonCurrTimetarget;
        #endif

        #if 0
        //写文件
        fwrite((char*)(&tmdata_out), tm_lenth, 1, fd_yc);
        fflush(fd_yc);
        #endif

      }
      else
      {


          if(bitmap_send_countck<bitmap_send_count)
           {
              SendTmData(&tmdata);

              ZhData(&tmdata,&tmdata_out);
              tm_lenth_bitmap=SendBitMapYc((&tmdata_out),yc_send);
              UART0_Send(fd_out,(char *)yc_send,tm_lenth_bitmap);
              bitmap_send_countck++;
              #if 0
              //写文件
              fwrite((char*)yc_send, tm_lenth_bitmap, 1, fd_yc);
              fflush(fd_yc);
             #endif
              printf("发送bitmap %d %d\n",tm_lenth_bitmap,bitmap_send_countck);
          }
          else
          {
              bitmap_send_flag=0;
          }

      }






        //写文件
       // fwrite((char*)(&tmdata), tm_lenth, 1, fd_yc);




        #ifdef USE_PRINTF_YC
        //printf("yc send 0k\n");
        #endif

        #if 0
        lonCurrTimetarget = getSystime(currTime);
        printf("lonCurrYC:%f\n",(lonCurrTimetarget-lonCurrTimetarget1)/1000.);
        lonCurrTimetarget1 =  lonCurrTimetarget;
        #endif

        //存储下文件

        pthread_mutex_unlock(&mutex);

        #endif




        #ifdef  IF_PRINTF_MONITOR
        #if 0
        lonCurrTimetarget = getSystime(currTime);
        printf("lonCurrYC:%f  %d %d\n",(lonCurrTimetarget-lonCurrTimetarget1)/1000.,lonCurrTimetarget,lonCurrTimetarget1 );
        lonCurrTimetarget1 =  lonCurrTimetarget;
        #endif
        #endif


     }
     else if(ins_typecode==0x2d)  //时间广播反馈
     {

            //网络传输过去

            //发送给image
            SendImageSoftWare(send_net,data+5,6,0x22);

            //先去掉
            setcalibrationTime(data+5);
            //更改方式  1970距离2017年1483142400

            //udpServer_sendDataEx(&server, ((char *)send_net), send_net_number, dstIpAddr, dst_port);
            #ifdef SEND_YC_YQ

            telctl_feedback(fd_out,0xaa,0xd2); //数据传输正确  (时间码12字节)

            #endif

           #ifdef  IF_PRINTF_MONITOR
           printf("广播：%x %x %x %x %x %x\n",*(send_net,data+5),*(send_net,data+6),\
                   *(send_net,data+7),*(send_net,data+8),*(send_net,data+9),*(send_net,data+10));
           #endif
           //ret=udpServer_sendDataEx(&server,(char*)(&tmdata),12,"127.0.0.3",6003);  //发送数

            #if 0
            lonCurrTimetarget = getSystime(currTime);
            printf("校时时差:%f  %d %d\n",(lonCurrTimetarget-lonCurrTimetarget1)/1000.,lonCurrTimetarget,lonCurrTimetarget1 );
            lonCurrTimetarget1 =  lonCurrTimetarget;
            #endif

      }




        //进入了default和else分支，也是遥控指令没有的情况
  } // if(checkSum(data+5,lenth,data[lenth+5])==SUCCESS)  //0xeb90的结构
    else
    {

         #ifdef SEND_YC_YQ

        if(ins_typecode==0x2d)
        {
          telctl_feedback(fd_out,0xff,0xd2); //数据传输错误
          tmdata.wrong_ins_count++;
          printf("时间戳校验和错误\n");
         }
        else if(ins_typecode==0x1e)
         {
            telctl_feedback(fd_out,0xff,0xe1); //数据传输错误
            tmdata.wrong_ins_count++;
            printf("遥控校验和错误\n");
        }
       #endif

    }
}

/*********************************************************

  1.初始化缓存
********************************************************/
void InitUartMemory()
{
   //
    unsigned int i;
    for(i=0;i<PORT_NUMBER;i++)
    {
        //data_uart_in[i]=new unsigned char[Port_Lenth];
        data_uart_process[i].m_nCur=0;
        data_uart_process[i].m_nEnd=0;

        if(i==0)  //反熔丝
        {
            data_uart_process[i].frame_head=new unsigned char[2];
            memcpy(data_uart_process[i].frame_head,frame_head_telctl,2);
            data_uart_process[i].frame_head_lenth=2;
            data_uart_process[i].refer_lenth=7;
             data_uart_process[i].uart_data=new unsigned char[Port_Lenth_Max];
        }
        else if(i>=1 && i<=6)  //相机帧头  6路相机
        {
            data_uart_process[i].frame_head=new unsigned char[2];
            memcpy(data_uart_process[i].frame_head,frame_head_camera,2);
            data_uart_process[i].frame_head_lenth=2;
            data_uart_process[i].refer_lenth=Refer_Lenth_Camera;
            data_uart_process[i].uart_data=new unsigned char[Port_Lenth];
        }
        /*
        else if(i==7)  //图像复接软件
        {
            data_uart_process[i].frame_head=new unsigned char[4];
            memcpy(data_uart_process[i].frame_head,frame_head_mutiple,4);
            data_uart_process[i].frame_head_lenth=4;
            data_uart_process[i].refer_lenth=Refer_Lenth;
        }
        else if(i==8)
        {

            data_uart_process[i].frame_head=new unsigned char[4];
            memcpy(data_uart_process[i].frame_head,frame_head_encode,4);
            data_uart_process[i].frame_head_lenth=2;
            data_uart_process[i].refer_lenth=7;
        }
        */

    }
}

/**************************************************************
 * 1.删除new开辟空间
 *
 *
 *
 * ************************************************************/
void DestroyUartMemory()
{
    unsigned int i;
    for(i=0;i<PORT_NUMBER;i++)
    {
        //delete[]data_uart_in[i];
        delete[]data_uart_process[i].frame_head;
        delete[]data_uart_process[i].uart_data;
     }
}



int set_uart(int* ptr_fd)
{
    /* 设置串口 */
    /* 1.获取串口属性 */
    struct termios termios_uart;
    memset(&termios_uart, 0, sizeof(termios_uart));
    int ret = tcgetattr(*ptr_fd, &termios_uart);
    if (ret == -1) {
        #ifdef  IF_PRINTF_MONITOR
        printf("tcgetattr failed\n");
        #endif
        return -1;
    }
    /* 2.配置串口 */
        //设置波特率
    cfsetispeed(&termios_uart,B115200);
    cfsetospeed(&termios_uart,B115200);
        //设置不进行校验
//    termios_uart.c_cflag &= ~PARENB;
//    termios_uart.c_iflag &= ~INPCK;
    termios_uart.c_cflag |= PARENB;
    termios_uart.c_cflag |= PARODD;
    termios_uart.c_iflag |= INPCK;
    termios_uart.c_iflag &= ~(ISTRIP);
        //设置数据位
    termios_uart.c_cflag &= ~CSIZE;
    termios_uart.c_cflag |= CS8;
    termios_uart.c_lflag&=~(ICANON | ECHO | ECHOE | ISIG);
        //设置串口为接收模式，本地连接模式
    termios_uart.c_cflag |= (CREAD | CLOCAL);
    //termios_uart.c_iflag &= termios_uart.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | INLCR | IXOFF);
    termios_uart.c_iflag &= termios_uart.c_iflag &= ~(BRKINT | ICRNL | IXON | INLCR | IXOFF);
    termios_uart.c_oflag &= ~(OPOST);
    //termios_uart.c_cc[VTIME] = 1;
    //termios_uart.c_cc[VMIN]  = 1;

    termios_uart.c_cc[VTIME] = 0;
    termios_uart.c_cc[VMIN]  = 7;

    tcflush (*ptr_fd, TCIFLUSH);
    /* 3.写入配置 */
    ret = tcsetattr(*ptr_fd, TCSANOW, &termios_uart);
    if (ret == -1) {
        #ifdef  IF_PRINTF_MONITOR
        printf("tcsetattr failed\n");
        #endif
        return -1;
    }
    tcflush (*ptr_fd, TCIOFLUSH);
    return 0;
}

int set_uart1(int* ptr_fd)
{
    /* 设置串口 */
    /* 1.获取串口属性 */
    struct termios termios_uart;
    memset(&termios_uart, 0, sizeof(termios_uart));
    int ret = tcgetattr(*ptr_fd, &termios_uart);
    if (ret == -1) {
        printf("tcgetattr failed\n");
        return -1;
    }
    /* 2.配置串口 */
        //设置波特率
    cfsetispeed(&termios_uart,B115200);
    cfsetospeed(&termios_uart,B115200);
        //设置不进行校验
    termios_uart.c_cflag &= ~PARENB;
    termios_uart.c_iflag &= ~INPCK;
    //termios_uart.c_cflag |= PARENB;
    //termios_uart.c_cflag |= PARODD;
    //termios_uart.c_iflag |= INPCK;
    //termios_uart.c_iflag &= ~(ISTRIP);
        //设置数据位
    termios_uart.c_cflag &= ~CSIZE;
    termios_uart.c_cflag |= CS8;
    termios_uart.c_lflag&=~(ICANON | ECHO | ECHOE | ISIG);
        //设置串口为接收模式，本地连接模式
    termios_uart.c_cflag |= (CREAD | CLOCAL);
    //termios_uart.c_iflag &= termios_uart.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON | INLCR | IXOFF);
    termios_uart.c_iflag &= termios_uart.c_iflag &= ~(BRKINT | ICRNL | IXON | INLCR | IXOFF);
    termios_uart.c_oflag &= ~(OPOST);
    termios_uart.c_cc[VTIME] = 0;
    //termios_uart.c_cc[VMIN]  = 5;
    termios_uart.c_cc[VMIN]  = 6;
    tcflush (*ptr_fd, TCIFLUSH);
    /* 3.写入配置 */
    ret = tcsetattr(*ptr_fd, TCSANOW, &termios_uart);
    if (ret == -1) {
        printf("tcsetattr failed\n");
        return -1;
    }
    tcflush (*ptr_fd, TCIOFLUSH);
    return 0;
}
//
/****************************************************************
 1.初始化串口
 * ***************************************************************/
void InitUart()
{
     //int err=0;
     unsigned int i;

     for(i=0;i<PORT_NUMBER;i++)
     //for(i=0;i<7;i++)
      {
             fd[i] = UART0_Open(fd[i],port[i]); //打开串口，返回文件描述符

             if(fd[i]==FALSE)
              {
                  printf(" %d %s open  com fail\n",i+1,port[i]);
                 // return ;
             }
             //init com
             /*
             err = UART0_Init(fd[i],115200,0,8,1,'O'); //奇校验
             if(err==FALSE)
              {
                 printf("  init  com fail\n");
                 return ;
             }
             */

             if(i==0)
              {
                 set_uart(&fd[i]);
               }
             else
             {
                set_uart1(&fd[i]);
             }
     }
}


#ifdef  USE_OWN_QUEUE
void FindFrameYk_QUEUE(const DataBuf& data_buf,ReceiveData *data_process)
 {
     unsigned int i;
     unsigned int frame_head_sum=0;
     unsigned int yk_lenth;
     unsigned char data_code=0;

    data_process->m_nCur = 0;

   if(data_process->m_nEnd+data_buf.m_len<=(Port_Lenth_Max))
   {

     memcpy(data_process->uart_data+data_process->m_nEnd, data_buf.m_data, data_buf.m_len);
     data_process->m_nEnd+=data_buf.m_len;
     g_mem_pool_uart[0].free(data_buf.m_data);
     //data_process->refer_lenth一帧的长度
     //是就要判断下长度为7,因为长度可变，最小为7
    for(;(data_process->m_nEnd-data_process->m_nCur) >=data_process->refer_lenth;)
     {

        frame_head_sum=0;
        for(i=0;i<data_process->frame_head_lenth;i++)
        {
            if((*(data_process->uart_data+data_process->m_nCur+i))==(data_process->frame_head[i]))
             {
                frame_head_sum++;
            }
        }

        data_code=(*(data_process->uart_data+data_process->m_nCur+2));
        //找到帧头
        if(frame_head_sum==data_process->frame_head_lenth &&((data_code==0x0f)|(data_code==0x1e)|(data_code==0x2d)))
         {

             yk_lenth=((*(data_process->uart_data+data_process->m_nCur+3)<<8)|(*(data_process->uart_data+data_process->m_nCur+4)));


             if(yk_lenth>473)   //有效包长度最大466+6   467+6   512
             {
                   data_process->m_nCur+=4; //过掉两帧头加帧长
                   #ifdef  IF_PRINTF_MONITOR
                   printf("遥控有效包长度过长\n");
                   #endif
              }
             else if((yk_lenth+6)>(data_process->m_nEnd-data_process->m_nCur))  //不满一帧
             {
                 //printf("收到的遥控有效包长度过短，累积数据\n");
                 break; //退出不做处理
             }
             else
             {
                 TelctlProcess(data_process->uart_data+data_process->m_nCur,fd[0]);
                 data_process->m_nCur+=(yk_lenth+6);
                 //data_process->m_nCur+=data_process->refer_lenth;
                  //printf("m_end :%d m_nCur:%d\n",data_process->m_nEnd,data_process->m_nCur);
             }


        }
        else
         {
            data_process->m_nCur++;
        }
     }

    if((data_process->m_nEnd-data_process->m_nCur)>=0)
    {

    //已经不满足一帧，移到开头
    if (data_process->m_nCur!=0)
     {
       for(i=0;i<(data_process->m_nEnd-data_process->m_nCur);i++)
       {
           (*(data_process->uart_data+i)) = (*(data_process->uart_data+data_process->m_nCur+i));
       }

       data_process->m_nEnd -=data_process->m_nCur;
     }
     data_process->m_nCur = 0;
    }
    else
    {
        data_process->m_nEnd=0;
        data_process->m_nCur = 0;
        //printf("m_nCur:%d m_end:%d",data_process->m_nCur,data_process->m_nEnd);
    }

    }  //if(data_process->m_nEnd+data_buf.m_len<=(Port_Lenth_Max))
    else
     {
         data_process->m_nCur = 0;
         data_process->m_nEnd=0;
         g_mem_pool_uart[0].free(data_buf.m_data);
         #ifdef  IF_PRINTF_MONITOR
         printf("接收遥控指令开辟缓存过小\n");
         #endif
    }
 }

/******************************************************
    找帧头
    1.data_in 收到的串口数
    2.data_in_lenth 收到的串口数据长度
    3.data_process处理数据
    4.flag
    [1]flag=0   接收到的3559编码遥测
    [2]flag=1   接收到的相机1遥测
    [3]flag=2   接收到的相机2遥测
    [4]flag=3   接收到的相机3遥测
    [5]flag=4   接收到的相机4遥测
    [4]flag=5   接收到的相机5遥测
    [5]flag=6   接收到的相机6遥测
*********************************************************/
void FindFrame_QUEUE(const DataBuf& data_buf,\
               ReceiveData *data_process,unsigned int flag)
{
    unsigned int i;
    unsigned int frame_head_sum=0;
    unsigned int number_bh=0;
    int ret;

   if(data_process->m_nEnd+data_buf.m_len<=(Port_Lenth))
   {

       memcpy(data_process->uart_data+data_process->m_nEnd, data_buf.m_data, data_buf.m_len);
       data_process->m_nEnd+=data_buf.m_len;
       g_mem_pool_uart[flag].free(data_buf.m_data);
        //data_process->refer_lenth一帧的长度
       for(;(data_process->m_nEnd-data_process->m_nCur) >=data_process->refer_lenth;)  //遥控如果是就要判断下长度
        {

           frame_head_sum=0;
           for(i=0;i<data_process->frame_head_lenth;i++)
           {
               if((*(data_process->uart_data+data_process->m_nCur+i))==(data_process->frame_head[i]))
                {
                   frame_head_sum++;
               }
           }

          //g_mem_pool_uart[flag].free(data_buf.m_data);

           //找到帧头
           if(frame_head_sum==data_process->frame_head_lenth)
            {

               //分别做处理
               /*
               if(flag==0)  //接收到3559编码遥测
                {

               }
               else if(flag==1 ||flag==2 ||flag==3 ||flag==4 || flag==5 ||flag==6)  //6路相机遥测更新
               {

               }
               */

               if(camera_setmode==0)
               {
                   camera_setmode=1;
               }


               //将相机的遥测黏贴到相应的位置
               //memcpy(tmdata.camera_data+(flag-1)*3,data_process->uart_data+data_process->m_nCur+2,3);
               memcpy(tmdata.camera_data+(flag-1)*4,data_process->uart_data+data_process->m_nCur+2,4);




               #ifdef   HandCom

               //printf("数据：0x%x 0x%x 0x%x 0x%x 0x%x\n",*(data_process->uart_data+data_process->m_nCur+0),*(data_process->uart_data+data_process->m_nCur+1),*(data_process->uart_data+data_process->m_nCur+2),*(data_process->uart_data+data_process->m_nCur+2)&0xc,flag);
               if(((*(data_process->uart_data+data_process->m_nCur+2))&0xc)==0x4)  //请求模式
               {



                   if(roll_flag_mode==0)
                   {
                       number_bh=GetNumber(mode_if_changesend,(flag-1));
                       if(number_bh==0)  //测量模式
                       {
                           ret=SendUartIns(fd[flag],2);  //告警
                           //printf("测量状态：%x %d\n",*(data_process->uart_data+data_process->m_nCur+2),flag);

                       }
                       else
                       {
                            ret=SendUartIns(fd[flag],3);//监视
                            //printf("监控状态：%x %d\n",*(data_process->uart_data+data_process->m_nCur+2),flag);
                       }
                    }
                   else
                   {
                       if(flag<5)
                       {
                           ret=SendUartIns(fd[flag],2);  //告警
                           //printf("测量状态：%x %d\n",*(data_process->uart_data+data_process->m_nCur+2),flag);
                       }
                   }


               }
              #endif



                data_process->m_nCur+=data_process->refer_lenth;
           }
           else
            {
               data_process->m_nCur++;
           }
    }

   //已经不满足一帧，移到开头
   if (data_process->m_nCur!=0)
    {
      for(i=0;i<(data_process->m_nEnd-data_process->m_nCur);i++)
      {
          (*(data_process->uart_data+i)) = (*(data_process->uart_data+data_process->m_nCur+i));
      }

      data_process->m_nEnd -=data_process->m_nCur;
    }
    data_process->m_nCur = 0;
   }
   else
   {
         #ifdef  IF_PRINTF_MONITOR
         printf("相机%d缓存过小\n",flag);
         #endif

   }
}
#endif




//与反熔丝通信的串口
#if 0
#if 1
FILE *fd_yc2;
unsigned flag_write2=0;
#endif
void *uart_recv_msg_write(void *arg)
{
    int len;
    unsigned char* rcv_buf = NULL;
    DataBuf t_data_buf;
    unsigned char data_in[UART_LENTH];

    #if 1

    if(flag_write2==0)
    {
       fd_yc2 = fopen("/mnt/yc2.dat", "wb");
       flag_write2=1;
     }
    #endif


    while (1) //循环读取数据
    {

            len = read(fd[0],data_in,UART_LENTH);

            #if 1
            //写文件
            fwrite((char*)(data_in), len, 1, fd_yc2);
            fflush(fd_yc2);
            #endif




              if(len>0 &&len<=UART_LENTH)
              {

                if(!g_uart_que[0].full())
                {

                    rcv_buf = g_mem_pool_uart[0].alloc();
                    memcpy(rcv_buf,data_in,len);
                    t_data_buf.m_data   = rcv_buf;
                    t_data_buf.m_len    = len;
                    g_uart_que[0].push(t_data_buf);
                    #ifdef  USE_PRINTF_COM0
                    printf("fd  len:%d\n",len);
                    #endif

                 }
                else
                {
                    printf("uart[0]缓存满\n");

                }


               // printf("fd  len:%d\n",len);
               }
              else if(len>UART_LENTH)
              {
                  printf("接收数据过多uart0 %d\n",len);
              }
              else
              {
                  usleep(10*1000);
                 // printf("未读到\n");
              }
               thread_flag_tj[0]++;

      }
}
#endif


//与反熔丝通信的串口

#if 0
FILE *fd_yc2;
unsigned flag_write2=0;
#endif
unsigned int lenth_com0=0;
void *uart_recv_msg_write(void *arg)
{
    int len=0;
    unsigned char* rcv_buf = NULL;
    DataBuf t_data_buf;
    unsigned char data_in[UART_LENTH*4];

    //unsigned int lenth_all=0;

    #if 0

    if(flag_write2==0)
    {
       fd_yc2 = fopen("/mnt/yc2.dat", "wb");
       flag_write2=1;
     }
    #endif




    while (1) //循环读取数据
    {

          #if 0
          len = read(fd[0],data_in,UART_LENTH_recv);
          if(len>0)
          {
              //写文件
              lenth_all+=len;
              printf("lenth_all:%d\n",lenth_all);
              fwrite((char*)(data_in), len, 1, fd_yc2);
              fflush(fd_yc2);
           }
          #endif



            if(lenth_com0<7)
            {
              len = read(fd[0],data_in+lenth_com0,UART_LENTH_recv);

              if(len>0)
               {
                 lenth_com0=lenth_com0+len;
                 //lenth_all+=len;
                 //printf("lenth_all:%d\n",lenth_all);
                 //printf("len:%d\n",len);
               }

            }
            else //if(lenth_com0<7)
            {
                if(lenth_com0>0 &&lenth_com0<=UART_LENTH)
                {

                  if(!g_uart_que[0].full())
                  {

                     // if(resetFactory_flag==0)
                      {
                          rcv_buf = g_mem_pool_uart[0].alloc();
                          memcpy(rcv_buf,data_in,lenth_com0);
                          t_data_buf.m_data   = rcv_buf;
                          t_data_buf.m_len    = lenth_com0;
                          g_uart_que[0].push(t_data_buf);
                          #ifdef  USE_PRINTF_COM0
                          printf("fd  lenth_com0:%d thread_flag_tj[0]:%d\n",lenth_com0,thread_flag_tj[0]);
                          #endif
                       }

                   }
                  else
                  {
                      #ifdef  IF_PRINTF_MONITOR
                      printf("uart[0]缓存满\n");
                      #endif

                  }

               }
              else if(lenth_com0>UART_LENTH)
              {
                  #ifdef  IF_PRINTF_MONITOR
                  printf("接收数据过多uart0 %d %d\n",len,lenth_com0);
                  #endif
              }
              #if 0
              //写文件
              fwrite((char*)(data_in), lenth_com0, 1, fd_yc2);
              fflush(fd_yc2);
              #endif
              lenth_com0=0;
            } //if(lenth_com0<7)



             if(len<=0)
              {
                  //usleep(10*1000);
                 usleep(10*1000);

              }
            //printf("thread_flag_tj[0] :%d \n",thread_flag_tj[0]);
             thread_flag_tj[0]++;

      }



}







/*

#if 1
FILE *fd_yc;
unsigned flag_write=0;
#endif
void *uart_recv_msg_write(void *arg)
{
    #ifdef  USE_OWN_QUEUE   //自己写的queue写法
    unsigned char* rcv_buf = NULL;
    DataBuf t_data_buf;
    #endif

    static unsigned int countnew=0;


    int   maxfd   = 0;
    int len,fd_result;
    fd_set recv_fds0;
    int  ret;
    struct timeval time;
    unsigned data_in[2048];

    #if 1

    if(flag_write==0)
    {
       fd_yc = fopen("/mnt/yc.dat", "wb");
       flag_write=1;
     }
    #endif

    while (1) //循环读取数据
    {

      //测发送 寻找最大的maxfd

       FD_ZERO(&recv_fds0);
       FD_SET(fd[0],&recv_fds0);
       maxfd=fd[0];

        time.tv_sec = 0;
        //time.tv_sec = 0;
        time.tv_usec = 200000; //100ms

        thread_flag_tj[0]++;

        //使用select实现串口的多路通信

        fd_result= select(maxfd +1,&recv_fds0,NULL,NULL,&time);
        if(fd_result < 0)
        {

            usleep(1000);
             //printf("fail\n");
            continue;
        }
        else if(fd_result == 0)
        {

            usleep(1000);
             //printf("fail1\n");
            continue;
        }
        else
         {
                if(FD_ISSET(fd[0], &recv_fds0))
                 {


                    #ifdef  USE_OWN_QUEUE   //queue写法
                    rcv_buf = g_mem_pool_uart[0].alloc();
                    len = read(fd[0],rcv_buf,1024);
                    t_data_buf.m_data   = rcv_buf;
                    t_data_buf.m_len    = len;
                    g_uart_que[0].push(t_data_buf);
                    #endif

                    #if 1
                    //写文件
                    fwrite((char*)(rcv_buf), len, 1, fd_yc);
                    fflush(fd_yc);
                    #endif
                    printf("fd[0]  len: %d  countnew:%d\n",len,countnew);

                    countnew++;

                    #if 0
                     len = read(fd[0],data_in,1024);
                     printf("fd[0]  len: %d  countnew:%d\n",len,countnew);
                     countnew++;
                    #endif

                     usleep(1000*10);


                }
            }

        }

       UART0_Close(fd[0]);
}
*/




//6路相机可以监视的方法
#if 0
void *uart_recv_msg_write_camera(void *arg)
{

    #ifdef  USE_OWN_QUEUE   //自己写的queue写法
    unsigned char* rcv_buf = NULL;
    DataBuf t_data_buf;
    #endif


    int   maxfd   = 6;
    //mutipel channel
    int len,fd_result;
    fd_set recv_fds;
    struct timeval time;
    int  ret;
    int i;

    while (1) //循环读取数据
    {

      //测发送 寻找最大的maxfd

       thread_flag_tj[1]++;

       FD_ZERO(&recv_fds);
       FD_SET(fd[1],&recv_fds);
       FD_SET(fd[2],&recv_fds);
       FD_SET(fd[3],&recv_fds);
       FD_SET(fd[4],&recv_fds);
       FD_SET(fd[5],&recv_fds);
       FD_SET(fd[6],&recv_fds);



       //找最大
       maxfd=fd[1];
       for(i=2;i<PORT_NUMBER;i++)
       {
           if(maxfd<fd[i])
           {
             maxfd=fd[i];
            }
       }


        time.tv_sec = 1;
        //time.tv_sec = 0;
        time.tv_usec = 0000;

        //使用select实现串口的多路通信

        fd_result= select(maxfd +1,&recv_fds,NULL,NULL,&time);
        if(fd_result < 0)
        {

            usleep(50*1000);
            continue;
        }
        else if(fd_result == 0)
        {

            usleep(50*1000);
            continue;
        }
        else
         {

            for(i=1;i<6;i++)
            {
                if(FD_ISSET(fd[i], &recv_fds))
                 {
                    #ifdef  USE_OWN_QUEUE   //queue写法
                    if(!g_uart_que[i].full())
                    {
                        rcv_buf = g_mem_pool_uart[i].alloc();
                        len = read(fd[i],rcv_buf,UART_LENTH);
                        t_data_buf.m_data   = rcv_buf;
                        t_data_buf.m_len    = len;
                        g_uart_que[i].push(t_data_buf);

                        printf("fd[%d]  len:%d\n",i,len);
                    }
                    else
                    {
                        printf("uart[i]缓存满\n",i);
                    }
                    #endif
                }
            }

        }


    }

   for(i=1;i<6;i++)
   {
      UART0_Close(fd[i]);
    }
}
#endif


void *uart_recv_msg_write_camera(void *arg)
{


    unsigned char* rcv_buf = NULL;
    DataBuf t_data_buf;
    int len;
    unsigned char data_in[UART_LENTH];




    int i;

    while (1) //循环读取数据
    {

      //测发送 寻找最大的maxfd

       thread_flag_tj[1]++;

       for(i=1;i<7;i++)
       {
           len = read(fd[i],data_in,UART_LENTH);

           if((len>0 &&len<=UART_LENTH))
           {

              #if 0
              printf("fd  len  i:%d  %d\n",len,i);
              for(int j=0;j<len;j++)
              {
                  printf("com[%d]:%x",i,data_in[j]);
              }

                printf("\n");
             #endif

             if(!g_uart_que[i].full())
             {
                 rcv_buf = g_mem_pool_uart[i].alloc();
                 memcpy(rcv_buf,data_in,len);
                 t_data_buf.m_data   = rcv_buf;
                 t_data_buf.m_len    = len;
                 g_uart_que[i].push(t_data_buf);
                // printf("fd  len  i:%d  %d\n",len,i);
              }
             else
             {
                 #ifdef  IF_PRINTF_MONITOR
                 printf("uart[i]缓存满\n");
                 #endif

             }
           }
          else if(len>UART_LENTH)
          {
              #ifdef  IF_PRINTF_MONITOR
              printf("接收数据过多uart[%d] %d\n",i,len);
              #endif
          }
      }  // for(i=1;i<7;i++)



       usleep(500*1000);
    }  //while

   for(i=1;i<6;i++)
   {
      UART0_Close(fd[i]);
    }
}


 /*********************************************************************************
  * 6路串口出栈，读数据
  *
  *
  *
  *
  * *******************************************************************************/
 void *uart_recv_msg_read(void *arg)
 {
     //unsigned int i;
     DataBuf t_data_buf;

     while(1)
     {
         #ifdef  USE_QUEUE
         FindFrameYk_QUEUE(q_uart,&data_uart_process[0]);
         FindFrameYk_QUEUE(q_cam1,&data_uart_process[1]);

         #endif

         #ifdef  USE_OWN_QUEUE
         if(!g_uart_que[0].empty())
         {
             //串口0
             t_data_buf = g_uart_que[0].pop();            
             //for example, use data_uart_process[0]
             FindFrameYk_QUEUE(t_data_buf, &data_uart_process[0]);
             #ifdef  USE_PRINTF_COM0
             printf("pop0\n");
             #endif
          }
         else
          {
              usleep(10*1000);
              //printf("thread_flag_tj[3] :%d bb\n",thread_flag_tj[3]);
         }

         thread_flag_tj[3]++;
         // printf("thread_flag_tj[3] :%d aa\n",thread_flag_tj[3]);

         #endif

     }

 }


 void *uart_recv_msg_read_camera(void *arg)
 {
     //unsigned int i;
     DataBuf t_data_buf;
     unsigned int count;

     while(1)
     {


         #ifdef  USE_OWN_QUEUE
         count=0;
         for(int i=1;i<7;i++)
          {
             //相机1
             if(!g_uart_que[i].empty())
             {
               t_data_buf = g_uart_que[i].pop();
               FindFrame_QUEUE(t_data_buf, &data_uart_process[i],(i));
               count++;
               #ifdef  USE_PRINTF_COM_CAMERA
               printf("camera pop%d\n",i);
               #endif

              }

         }
         thread_flag_tj[2]++;

         usleep(500*1000);


         #endif


     }

 }





