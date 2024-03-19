#ifndef RECEIVE_UDP_H
#define RECEIVE_UDP_H


#ifdef __cplusplus
extern "C" {
#endif
#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*POSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <string.h>       /*字符串功能函数*/


#include    <pthread.h>
#include    <sched.h>
#include "./udpServer/udpServerNormal.h"



//遥测量结构体
 #pragma pack(1)

#pragma pack()





#define BUF_SIZE  2048
extern udpServer server;
extern void *udp_recv_msg(void *arg);
extern void InitServerUdp();

extern void SendDataExposureMain(unsigned int number,int fd_send);
extern unsigned long getCpuMs();

#ifdef TEST_DELAY
extern unsigned int  delay_flag_udp;
#endif
#ifdef TEST_DELAY_A
extern unsigned int  delay_flag_udp;
#endif



//宏定义
#define FALSE  -1
#define TRUE   0

#ifdef __cplusplus
}
#endif


#endif // RECEIVE_UDP_H
