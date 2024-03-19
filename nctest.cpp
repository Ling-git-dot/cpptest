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

#define PORT_NUMBER 7
#define Port_Lenth  2048
#define Refer_Lenth_Camera 5
#define Port_Lenth_Max 4*2048*2048

const char *port[]={"/dev/ttyUL1","/dev/ttyUL2","/dev/ttyUL3","/dev/ttyUL4","/dev/ttyUL5","/dev/ttyUL6","/dev/ttyUL7"};

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


//unsigned char mode_if_changesend[6]={2,4,0,1,3,5};  //IDS更改后的 广域c 远场 广域a  广域b 广域d  近场
unsigned char mode_if_changesend[6]={4,5,0,1,2,3};


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