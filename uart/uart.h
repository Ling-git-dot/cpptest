#ifndef UART_H
#define UART_H

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

#define SUCCESS 0xAA
#define ERROR 0xFF


int UART0_Open(int fd,const char* port);
void UART0_Close(int fd);
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART0_Recv(int fd, char *rcv_buf,int data_len);
int UART0_Send(int fd, char *send_buf,int data_len);
void *uart_recv_msg(void *arg);


//宏定义
#define FALSE  -1
#define TRUE   0

#ifdef __cplusplus
}
#endif
#endif// UART_H

