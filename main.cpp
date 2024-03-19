#include "receive_uart.h"
#include "receive_udp.h"
//#include "./gpio/gpioset.h"
#include "online_program.h"

#if 0
#include <map>
#include <string.h>
std::map<std::string, std::string>  t_test_map1;
#endif

//void InitGpio()
//{
//    unsigned int i;
//    for(i=4;i<10;i++)
//    {
//        initGpio(i);
//        setGpioDirection(i,(char*)("out"));
//        //setGpioValue()

//    }

//}

//void InitGpioout(unsigned int flag,unsigned int channel)
//{
//    unsigned int i;
//    /*
//    for(i=0;i<32;i++)
//    {
//        initGpio(32+i);
//        setGpioDirection(32+i,(char*)("in"));
//    }
//    */
//   // for(i=4;i<10;i++)
//    //{
//        //initGpio(i);
//        //setGpioDirection(i,(char*)("out"));
//        //setGpioValue()
//        //gpio_level_set(i,flag);
//   // }

//    gpio_level_set(channel,flag);



//}





//unsigned long getCpuMs()
//{
//    unsigned long mCpuTickMs = 0;
//    timeval tmp;
//    gettimeofday(&tmp,NULL);
//    mCpuTickMs = tmp.tv_sec*1000+tmp.tv_usec/1000;
//    return mCpuTickMs;
//}



unsigned int camera_flag_set=0;

#ifdef GPIO_DOG
unsigned int gpio_number=0;

#endif

int main(int argc, char *argv[])
{

   //char  *image_argv = argv[0];
   unsigned int i;

    #ifdef SEND_YC_TIME
    unsigned  int tm_lenth;
    unsigned int tm_lenth_bitmap;
    #endif

    #ifdef  TEST_DELAY

    long lonCurrTimetarget_before;
    long lonCurrTimetarget;
    unsigned char currTime[6] = {0};

    #endif

   printf("monitor begin\n");

   #if 0
   //str to char  实现文件的copy覆盖
   char kk[200]={0};
   char aa[200]="/home/qnh/Downloads/logzg/lib_backup/2";
   char bb[200]="/home/qnh/Downloads/logzg/lib_dir/2";
   char cc1[200]="/usr/lib/b.so";
   char dd1[200]="/usr/b.so";
   char tt[200];

   while (!t_test_map1.empty())
   {

       t_test_map1.erase(t_test_map1.begin());
   }
   std::string cc;
   t_test_map1[bb]=aa;
   t_test_map1[dd1]=cc1;
   cc=t_test_map1[bb];
   strcpy(tt, cc.c_str());

   char des[500] = {"cp "};
   strncat(des, aa, 200);
   strncat(des, " ", 10);
   strncat(des, bb, 200);

   ExecuteCMD(des,kk);

   while (!t_test_map1.empty())
   {

       t_test_map1.erase(t_test_map1.begin());
   }
   #endif




   //读取文件
   #ifdef  USE_ZAIGUI

   getLog(LOG_ROUTE, sizeof(LOG_ROUTE));

   //将现在有的状态读取出来
   if(log_data.flag_online==0xaa)
   {
       zg_status=0x1;
       enProgramModify=0xaa;  //保护指令状态
   }
   else
   {
       zg_status=0x0;
       enProgramModify=0xff;  //保护指令状态
   }

   if(log_data.flag_update==0xff)
   {
       zg_excut=0x1;
   }
   else
   {
       zg_excut=0x0;
   }
   //zg_rightcount=log_data.zg_rightcount;
   //zg_wrongcount=log_data.zg_wrongcount;
   #endif


  // InitGpioIn();   //暂时不用gpio发指令
   InitGpio();



   for(i=4;i<10;i++)
   {
      InitGpioout(0,i);
   }
   sleep(2);


   //2023.12.21

//   for(i=4;i<10;i++)
//   {
//      InitGpioout(1,i);
//   }




   //sleep(5);
   //SetCameraMode();


   //unsigned int a=getGpioValueMutiple(32,32); //暂时不用gpio发指令





    //setbuf(stdout, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);

   // InitTmData(&tmdata);

    //初始化互斥锁（串口更新遥测时应用）
    pthread_mutex_init(&mutex, NULL); //默认使用快速互斥锁




    InitServerUdp(); //建立网络客户端



    //初始化缓存
    InitUartMemory();



    // init com 初始化串口
    InitUart();
   // set_uart(&fd[0]);
    //set_uart(&fd[1]);
    //set_uart(&fd[2]);
    //set_uart(&fd[3]);
    //set_uart(&fd[4]);
   // set_uart(&fd[5]);
    //set_uart(&fd[6]);



    //遥测初始化
    InitTmData(&tmdata);


    //初始化shenzong遥测
    InitBramMonitor();

    //初始化clock核
    InitBramClock();



    //写程序
    ModifyFjyk();

    SetWorkmode(tmdata.work_mode);

    //局部曝光增加参数
    memset(data_exposure_flag,0,6);





    //初始化串口接收需要的缓存
    #ifdef  USE_QUEUE_THREAD
    //set thread uart write(监测网口)
    pthread_t tid_uart_write;
    struct sched_param param_uart_receive_write;
    if(pthread_create(&tid_uart_write,NULL,uart_recv_msg_write,NULL)<0)
    {
        printf("thread wrong uart_recv_msg_write\n");
        return -1;
    }
    param_uart_receive_write.sched_priority = 10;//优先级1~99 1为最低值
    pthread_setschedparam(tid_uart_write, SCHED_OTHER, &param_uart_receive_write);
    pthread_detach(tid_uart_write);





    //set thread uart read
    pthread_t tid_uart_read;
    struct sched_param param_uart_receive_read;
    if(pthread_create(&tid_uart_read,NULL,uart_recv_msg_read,NULL)<0)
    {
        printf("thread wrong uart_recv_msg_read\n");
        return -1;
    }
    param_uart_receive_read.sched_priority = 10;//优先级1~99 1为最低值
    pthread_setschedparam(tid_uart_read, SCHED_OTHER, &param_uart_receive_read);
    pthread_detach(tid_uart_read);


    //与6个相机串口通信  单独两个线程，不要影响遥控线程

    #if 1
    pthread_t tid_uart_write_camera;
    struct sched_param param_uart_receive_write_camera;
    if(pthread_create(&tid_uart_write_camera,NULL,uart_recv_msg_write_camera,NULL)<0)
    {
        printf("thread wrong param_uart_receive_write_camera\n");
         return -1;
    }
    param_uart_receive_write_camera.sched_priority = 10;//优先级1~99 1为最低值
    pthread_setschedparam(tid_uart_write_camera, SCHED_OTHER, &param_uart_receive_write_camera);
    pthread_detach(tid_uart_write_camera);
    #endif

    #if 1
    pthread_t tid_uart_read_camera;
    struct sched_param param_uart_receive_read_camera;
    if(pthread_create(&tid_uart_read_camera,NULL,uart_recv_msg_read_camera,NULL)<0)
    {
        printf("thread wrong uart_recv_msg_read\n");
        return -1;
    }
    param_uart_receive_read_camera.sched_priority = 10;//优先级1~99 1为最低值
    pthread_setschedparam(tid_uart_read_camera, SCHED_OTHER, &param_uart_receive_read_camera);
    pthread_detach(tid_uart_read_camera);
    #endif

    #endif



    //set thread receive udp data   建立网口接收线程
    #if 1
    pthread_t tid_udp;
    struct sched_param param_udp_receive;
    if(pthread_create(&tid_udp,NULL,udp_recv_msg,NULL)<0)
    {
        printf("thread wrong");
         return -1;
    }
    param_udp_receive.sched_priority = 10;
    pthread_setschedparam(tid_udp, SCHED_OTHER, &param_udp_receive);
    pthread_detach(tid_udp);
    #endif



   //int status;
   //unsigned char data[5]={0xaa,0x90,0xaa,0xbb,0xcc};

  #if 0
   static unsigned int flag_write1=0;
   FILE *fd_yc1;
   if(flag_write1==0)
   {
      fd_yc1 = fopen("/mnt/yc1.dat", "wb");
      flag_write1=1;
    }
  #endif


   //曝光可以设置在这
   #ifdef  SetIntervalExplore
   unsigned long  currCpuTickpoll=0;
   unsigned long LastCpuTickpoll=0;
   unsigned long calibMspoll = 0;
   LastCpuTickpoll=getCpuMs();
   #endif


   while(1)
   {

       thread_flag_tj[5]++;

       if(camera_setmode==1 && camera_flag_set==0)
        {

           //SetCameraMode();
           camera_flag_set=1;

           #ifdef IF_PRINTF_MONITOR
           printf("更新完成\n");
           #endif
       }


       //usleep(20000);
       //usleep(50*1000);

       #ifdef USE_MAINTHREAD
       printf("thread_flag_tj[0]:%d thread_flag_tj[1]:%d\n",thread_flag_tj[0],thread_flag_tj[1]);
       #endif

       #ifdef  SEND_YC_TIME
       /*
       unsigned int lenth_tm=sizeof(tmPackage);
       SendTmData(&tmdata);

       ZhData(&tmdata,&tmdata_out);

       UART0_Send(fd[0],(char*)(&tmdata_out),sizeof(tmPackage));
       */



       #ifdef  SetIntervalExplore
       currCpuTickpoll=getCpuMs();

       if(currCpuTickpoll>=LastCpuTickpoll)
        {
           calibMspoll = currCpuTickpoll - LastCpuTickpoll;
       }
       else
        {
           calibMspoll = (0xFFFFFFFFFFFFFFFF - LastCpuTickpoll + currCpuTickpoll);
       }

       //if(calibMs>60000)  //60*1000ms
       if(calibMspoll>500)  //60*1000ms
        {
           //定时切换指令  每隔1s钟
           LastCpuTickpoll=currCpuTickpoll;

           for(i=0;i<6;i++)
           {
               if(data_exposure_flag[i]==1)
                {
                  SendDataExposureMain(i,fd[i+1]);


                }
           }
       }
       #endif


       if(if_send_yc_image==0 &&if_send_yc_monitor==0)   //2023.11.9 线程是否存活状态  喂狗
       {
           if(bitmap_send_flag==0)
            {


             tm_lenth=SendTmData(&tmdata);


             #if 0
             setcalibrationTime(data);
             #endif


             ZhData(&tmdata,&tmdata_out);
             UART0_Send(fd[0],(char*)(&tmdata_out),tm_lenth);

            #ifdef  TEST_DELAY
            if(tmdata.target_frame_data.ditance_data[1]==0x73  &&tmdata.target_frame_data.ditance_data[2]==0xd7&& delay_flag_main==0)
            {
                lonCurrTimetarget = getSystime(currTime);
                delay_flag_main=1;
                //printf("delay_flag_main:%d\n",delay_flag_main);
                lonCurrTimetarget_before=(tmdata.target_frame_data.timesamp[0]|(tmdata.target_frame_data.timesamp[1]<<8)|(tmdata.target_frame_data.timesamp[2]<<16)|(tmdata.target_frame_data.timesamp[3]<<24)|(tmdata.target_frame_data.timesamp[4]<<32)|(tmdata.target_frame_data.timesamp[5]<<40));
                //printf("tmdata.target_frame_data:%x %x %x %x",tmdata.target_frame_data.timesamp[0],tmdata.target_frame_data.timesamp[1],tmdata.target_frame_data.timesamp[2],tmdata.target_frame_data.timesamp[3]);
                printf("采图时间：%d %d %d\n",lonCurrTimetarget,lonCurrTimetarget_before,(lonCurrTimetarget-lonCurrTimetarget_before));
            }
            #endif

             #if 0
             //写文件
             fwrite((char*)(&tmdata_out), tm_lenth, 1, fd_yc1);
             fflush(fd_yc1);
             #endif

              usleep(50*1000);

           }
           else
           {


               if(bitmap_send_countck<bitmap_send_count)
                {
                   SendTmData(&tmdata);

                   ZhData(&tmdata,&tmdata_out);
                   tm_lenth_bitmap=SendBitMapYc((&tmdata_out),yc_send);
                   UART0_Send(fd[0],(char *)yc_send,tm_lenth_bitmap);
                   bitmap_send_countck++;
                   #if 0
                   //写文件
                   fwrite((char*)yc_send, tm_lenth_bitmap, 1, fd_yc1);
                   fflush(fd_yc1);
                  #endif
                   //printf("发送bitmap %d %d\n",tm_lenth_bitmap,bitmap_send_countck);
               }
               else
               {
                   bitmap_send_flag=0;
               }

                usleep(500*1000);

           }
        }


       //增加喂狗  检查线程是否存在的函数
       //*pthread_kill的返回值：成功（0） 线程不存在（ESRCH） 信号不合法（EINVAL）*/
       //ESRCH		 3	/* No such process */
       //#define	EINVAL		22	/* Invalid argument */
       //是否活着还是要不堵在那  还是用帧计数的方式




       #ifdef GPIO_DOG
       //printf("kill_rc %d %d %d %d %d\n",pthread_kill(tid_uart_write,0),pthread_kill(tid_uart_read,0),pthread_kill(tid_uart_write_camera,0),pthread_kill(tid_uart_read_camera,0),pthread_kill(tid_udp,0));

       //pthread_cancel(tid_uart_write);  //测试用

       if(pthread_kill(tid_uart_write,0)==0 &&pthread_kill(tid_uart_read,0)==0 && pthread_kill(tid_uart_write_camera,0)==0&& pthread_kill(tid_udp,0)==0)
       {

//           if( gpio_number%2==0)
//            {
//                InitGpioout(0,10);
//           }
//           else
//            {
//               InitGpioout(1,10);
//           }

//           printf("thread live\n");
//           gpio_number++;

           if_send_yc_monitor=0;


       }
       else
       {
           if_send_yc_monitor=1;
       }


       #endif






       #endif

      #if 0
       //fwrite((char*)(&tmdata_out), sizeof(tmPackage), 1, fd_yc1);
       //fflush(fd_yc1);
      #endif
/*
       status=UART0_Send(fd[0],(char *)data,5);
       if(status!=-1)
       {
           printf("fd[0] status %d\n",status);
       }
       else
        {
           printf("error\n");
       }
     */


        /*
       usleep(10000);
       status=UART0_Send(fd[1],(char *)data,5);
       if(status!=-1)
       {
           printf("fd[1] status %d\n",status);
       }
       else
        {
           printf("error\n");
       }

       usleep(10000);
       status=UART0_Send(fd[2],(char *)data,5);
       if(status!=-1)
       {
           printf("fd[2] status %d\n",status);
       }
       else
        {
           printf("error\n");
       }
       usleep(10000);
       status=UART0_Send(fd[3],(char *)data,5);
       if(status!=-1)
       {
           printf("fd[3] status %d\n",status);
       }
       else
        {
           printf("error\n");
       }
       usleep(10000);
       status=UART0_Send(fd[4],(char *)data,5);
       if(status!=-1)
       {
           printf("fd[4] status %d\n",status);
       }
       else
        {
           printf("error\n");
       }
       usleep(10000);
       status=UART0_Send(fd[5],(char *)data,5);
       if(status!=-1)
       {
           printf("fd[5] status %d\n",status);
       }
       else
        {
           printf("error\n");
       }
       usleep(10000);
       status=UART0_Send(fd[6],(char *)data,5);
       if(status!=-1)
       {
           printf("fd[6] status %d\n",status);
       }
       else
        {
           printf("error\n");
       }
       */


   }


    return 0;
}
