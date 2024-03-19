#include "receive_uart.h"
#include "receive_udp.h"

udpServer server;

unsigned char image_process_recv_buffer[BUF_SIZE] = {0};



#ifdef TEST_DELAY
unsigned int  delay_flag_udp=0;
#endif

#ifdef TEST_DELAY_A
unsigned int  delay_flag_udp=0;
#endif
/************************************************************
  1.初始化网络
  2.建立server服务器 udp
 ***********************************************************/
void InitServerUdp()
{
    //int ret;
    if(setUdpServerEx(&server,"127.0.0.2", 6002) != UDP_SUCCESS)
    //if(setUdpServerEx(&server,"192.168.1.90", 6002) != UDP_SUCCESS)
    //if(setUdpServerEx(&server,localIpAddr, local_port) != UDP_SUCCESS)
    {
         #ifdef IF_PRINTF_MONITOR
          printf("server udp init fail\n");
          #endif
    }

   if(createUdpServer(&server) != UDP_SUCCESS)
   {
       udpServer_showErr(&server);
        #ifdef IF_PRINTF_MONITOR
       printf("server udp  fail\n");
       #endif
   }

}

#ifdef  TEST_DELAY
long getSystimeudp(unsigned char time[6])
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
#endif

#ifdef  SetIntervalExplore
void SendDataExposureMain(unsigned int number,int fd_send)
{
    int status;

    status=UART0_Send(fd_send,(char *)(&(data_exposure[number][0])),12);

}
#endif

void  SendDataExposure(unsigned int number,unsigned char *data_in,int fd_send)
{
    unsigned char sum=0;
    unsigned int i;
    int status;
   // unsigned char data_data[12];


   if(number==0 || number==1 ||number==2 ||number==3)
   {
       data_exposure[number][0]=0xeb;
       data_exposure[number][1]=0x90;
       data_exposure[number][2]=0xFE;
       data_exposure[number][3]=0;
       data_exposure[number][4]=0;
       data_exposure[number][5]=0;
       data_exposure[number][6]=0;
       data_exposure[number][7]=0x08;
       data_exposure[number][8]=0;  //2048  0x800
       data_exposure[number][9]=0x08;
       data_exposure[number][10]=0;  //2048  0x800
       for(i=0;i<11;i++)
       {
           sum+=data_exposure[number][i];
       }
       data_exposure[number][11]=sum;
   }
   else
    {
      //高字节在前，高bit在前
       data_exposure[number][0]=0xeb;
       data_exposure[number][1]=0x90;
       data_exposure[number][2]=0xFE;
//       data_exposure[number][3]=data_in[0];
//       data_exposure[number][4]=data_in[1];
//       data_exposure[number][5]=data_in[2];
//       data_exposure[number][6]=data_in[3];
//       data_exposure[number][7]=data_in[4];
//       data_exposure[number][8]=data_in[5];
//       data_exposure[number][9]=data_in[6];
//       data_exposure[number][10]=data_in[7];


      data_exposure[number][3]=data_in[1];
      data_exposure[number][4]=data_in[0];
      data_exposure[number][5]=data_in[3];
      data_exposure[number][6]=data_in[2];
      data_exposure[number][7]=data_in[5];
      data_exposure[number][8]=data_in[4];
      data_exposure[number][9]=data_in[7];
      data_exposure[number][10]=data_in[6];
       for(i=0;i<11;i++)
       {
           sum+=data_exposure[number][i];
       }
       data_exposure[number][11]=sum;

   }


//   printf("x: %x %x\n",data_exposure[number][3],data_exposure[number][4]);  //[3]低字节  [4]低字节
//   printf("y: %x %x\n",data_exposure[number][5],data_exposure[number][6]);
//   printf("lenth: %x %x\n",data_exposure[number][7],data_exposure[number][8]);  //[8]是高字节
//   printf("width: %x %x\n",data_exposure[number][9],data_exposure[number][10]);


   #ifndef  SetIntervalExplore
  // memcpy(data_data,&(data_exposure[number][0]),12);

   //printf("%x %x %x %x %x %x %x %x %x %x %x %x\n",data_data[0],data_data[1],data_data[2],data_data[3], data_data[4],data_data[5],data_data[6],data_data[7],data_data[8],data_data[9],data_data[10],data_data[11]);
   status=UART0_Send(fd_send,(char *)(&(data_exposure[number][0])),12);
   #endif

}

unsigned long getCpuMs()
{
    unsigned long mCpuTickMs = 0;
    timeval tmp;
    //gettimeofday(&tmp,NULL);
    mCpuTickMs = tmp.tv_sec*1000+tmp.tv_usec/1000;
    return mCpuTickMs;
}
/************************************************************
  1.建立与图像处理软件的通信
  接收图像传输过的遥测信息，并更新入遥测
 * **********************************************************/
void *udp_recv_msg(void *arg)
{
    int ret;
    unsigned int head=0;
    //unsigned short datalen;
    unsigned char checkstatus=0;
    unsigned char thread_status_zj[Image_Thread_NUMBER];

    //局部曝光增加参数
    unsigned int i;
    unsigned char data_exposure_temp[8];

    //曝光可以设置在这
    #if  1
    unsigned long  currCpuTickpoll=0;
    unsigned long LastCpuTickpoll=0;
    unsigned long calibMspoll = 0;
    LastCpuTickpoll=getCpuMs();
    #endif

#ifdef TEST_DELAY
    long  lonCurrTimetargetudp;
     unsigned char currTime[6] = {0};
     long lonCurrTimetargetudp_before=0;
#endif


#if 0
    static unsigned long time_data1;
    static unsigned long time_data1_temp;
    float time_data1cz;
    static unsigned time_data1_flag=0;



  #endif
    while(1)
    {

       thread_flag_tj[4]++;
       usleep(1000);
       memset(image_process_recv_buffer, 0, BUF_SIZE);
       ret = udpServer_waitData_timeout(&server, (char *)image_process_recv_buffer, BUF_SIZE, 1, 0);
       if(ret == UDP_FAIL)
       {
          udpServer_showErr(&server);
          #ifdef IF_PRINTF_MONITOR
          printf("error\n");
          #endif
          //continue;
       }
       else if(ret == UDP_TIMEOUT)
       {
          //printf("Timeout\n");
          //continue;
       }
       else
        {

           //printf("receive :%d\n",ret);
           //收到内部图像处理软件传输过来的图像遥测，更新遥测
           if ( (ret == sizeof(struct Send_Monitor_frame)))
           {
               // 高低字节转换  ntohl  ntohs

               head = ((image_process_recv_buffer[3] << 24) | (image_process_recv_buffer[2] << 16) |(image_process_recv_buffer[1] << 8) |(image_process_recv_buffer[0]));
               //datalen =((image_process_recv_buffer[4] << 8)|(image_process_recv_buffer[5]));
               checkstatus=checkSum1((unsigned char *)image_process_recv_buffer, sizeof(struct Send_Monitor_frame)-1, image_process_recv_buffer[sizeof(struct Send_Monitor_frame)-1]);
               if ((head == 0x1ACFFC1D) && checkstatus==0xaa)
               {
                   memcpy(&tmdata.target_frame_data,image_process_recv_buffer+4,sizeof(struct Image_frame)-9);
                   memcpy(thread_status_zj,image_process_recv_buffer+4+sizeof(struct Image_frame)-Image_Thread_NUMBER,Image_Thread_NUMBER);
                   //将线程帧计数更新进去
                   tmdata.thread_flag[0]=(((thread_status_zj[0]&0xf)<<4)|(thread_status_zj[1]&0xf));
                   tmdata.thread_flag[1]=(((thread_status_zj[2]&0xf)<<4)|(thread_status_zj[3]&0xf));
                   tmdata.thread_flag[2]=(((thread_status_zj[4]&0xf)<<4)|(thread_status_zj[5]&0xf));
                   tmdata.thread_flag[3]=(((thread_status_zj[6]&0xf)<<4)|(thread_status_zj[7]&0xf));
                   tmdata.thread_flag[4]&=0xf;
                   tmdata.thread_flag[4]|=((thread_status_zj[8]&0xf)<<4);
                  // memcpy(mode_if_change,(unsigned char *)image_process_recv_buffer+sizeof(struct Send_Monitor_frame)-7,6);
                  // printf("mode_if_change %x %x %x %x %x %x",mode_if_change[0],mode_if_change[1],mode_if_change[2],mode_if_change[3],mode_if_change[4],mode_if_change[5]);
                  //printf("net receive finish\n");

                 // printf("时间：%x %x %x %x %x %x\n",tmdata.target_frame_data.timesamp[0],tmdata.target_frame_data.timesamp[1],\
                          tmdata.target_frame_data.timesamp[2],tmdata.target_frame_data.timesamp[3],\
                          tmdata.target_frame_data.timesamp[4],tmdata.target_frame_data.timesamp[5]);

                    #ifdef  TEST_DELAY
                     if(tmdata.target_frame_data.ditance_data[1]==0x73  &&tmdata.target_frame_data.ditance_data[2]==0xd7 && delay_flag_udp==0)
                      {
                            lonCurrTimetargetudp = getSystimeudp(currTime);
                            lonCurrTimetargetudp_before=(tmdata.target_frame_data.timesamp[0]|(tmdata.target_frame_data.timesamp[1]<<8)|(tmdata.target_frame_data.timesamp[2]<<16)|(tmdata.target_frame_data.timesamp[3]<<24)|(tmdata.target_frame_data.timesamp[4]<<32)|(tmdata.target_frame_data.timesamp[5]<<40));
                            //printf("tmdata.target_frame_data:%x %x %x %x",tmdata.target_frame_data.timesamp[0],tmdata.target_frame_data.timesamp[1],tmdata.target_frame_data.timesamp[2],tmdata.target_frame_data.timesamp[3]);
                            printf("网络收图时间：%d,%d 差值：%d\n",lonCurrTimetargetudp,lonCurrTimetargetudp_before,(lonCurrTimetargetudp-lonCurrTimetargetudp_before));
                            //printf("lonCurrTimetargetudp_before:%x\n",lonCurrTimetargetudp_before);
                            delay_flag_udp=1;
                     }
                     #endif





                          #if  0
                              time_data1=((tmdata.target_frame_data.timesamp[0]<<24)|(tmdata.target_frame_data.timesamp[1]<<16)|(tmdata.target_frame_data.timesamp[2]<<8)|(tmdata.target_frame_data.timesamp[3]))*10000+\
                                      ((tmdata.target_frame_data.timesamp[4]<<8)|tmdata.target_frame_data.timesamp[5]);

                              if(time_data1_flag==1)
                              {
                                  time_data1cz= ((time_data1- time_data1_temp)/10000.);
                                  if(time_data1cz>1.5)
                                   {
                                      printf("time_data1cznet:%f\n",time_data1cz);
                                   }

                              }
                              time_data1_temp=time_data1;
                              time_data1_flag=1;
                          #endif

                  //局部曝光增加参数
                  #ifdef  GPIO_DOG
                        //if_send_yc_image=image_process_recv_buffer[sizeof(struct Send_Monitor_frame)-2];  //图像线程是否存在的状态  //局部曝光增加参数删除
                        if_send_yc_image=image_process_recv_buffer[4+sizeof(struct Image_frame)];  //图像线程是否存在的状态  //局部曝光增加参数删除
                  #endif

//                    printf("%x %x\n",*(image_process_recv_buffer+4+sizeof(struct Image_frame)+1),*(image_process_recv_buffer+4+sizeof(struct Image_frame)+2));
//                    printf("x:%x %x\n",*(image_process_recv_buffer+4+sizeof(struct Image_frame)+3),*(image_process_recv_buffer+4+sizeof(struct Image_frame)+4));
//                    printf("y:%x %x\n",*(image_process_recv_buffer+4+sizeof(struct Image_frame)+5),*(image_process_recv_buffer+4+sizeof(struct Image_frame)+6));
//                    printf("lenth:%x %x\n",*(image_process_recv_buffer+4+sizeof(struct Image_frame)+7),*(image_process_recv_buffer+4+sizeof(struct Image_frame)+8));
//                    printf("width:%x %x\n",*(image_process_recv_buffer+4+sizeof(struct Image_frame)+9),*(image_process_recv_buffer+4+sizeof(struct Image_frame)+10));
                   //局部曝光增加参数
                    #ifdef IF_EXPOSURE
//                    for(i=0;i<6;i++)
//                    {
//                        if(data_exposure_flag[i]==1)
//                         {
//                           if(i==4)
//                           {
//                               if(image_process_recv_buffer[4+sizeof(struct Image_frame)+1]==1)
//                               {
//                                  memcpy(data_exposure_temp,image_process_recv_buffer+4+sizeof(struct Image_frame)+3,8);
//                                  SendDataExposure(i,data_exposure_temp,fd[i+1]);
//                               }
//                           }
//                           else if(i==5)
//                           {
//                               if(image_process_recv_buffer[4+sizeof(struct Image_frame)+2]==1)
//                               {
//                                  memcpy(data_exposure_temp,image_process_recv_buffer+4+sizeof(struct Image_frame)+3,8);
//                                  SendDataExposure(i,data_exposure_temp,fd[i+1]);
//                               }
//                           }
//                           else
//                           {
//                               SendDataExposure(i,data_exposure_temp,fd[i+1]);
//                           }


//                         }
//                    }
                   #endif


                    #if 1
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
                               if(i==4)
                               {
                                   if(image_process_recv_buffer[4+sizeof(struct Image_frame)+1]==1)
                                   {
                                      memcpy(data_exposure_temp,image_process_recv_buffer+4+sizeof(struct Image_frame)+3,8);
                                      SendDataExposure(i,data_exposure_temp,fd[i+1]);
                                   }
                               }
                               else if(i==5)
                               {
                                   if(image_process_recv_buffer[4+sizeof(struct Image_frame)+2]==1)
                                   {
                                      memcpy(data_exposure_temp,image_process_recv_buffer+4+sizeof(struct Image_frame)+3,8);
                                      SendDataExposure(i,data_exposure_temp,fd[i+1]);
                                   }
                               }
                               else
                               {
                                   SendDataExposure(i,data_exposure_temp,fd[i+1]);
                               }


                             }
                        }
                    }
                    #endif



               }
               else
               {
                  //printf("receive  udp wrong\n");
               }
           }
           else
            {
               #ifdef IF_PRINTF_MONITOR
               printf("receive  udp wrong lenth %d\n",ret);
               #endif
           }
       }

        //udpServer_sendDataEx(&server, image_process_recv_buffer, 16, "127.0.0.3", 6003);


    }

}
