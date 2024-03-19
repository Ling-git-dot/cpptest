#include <stdio.h>
 
#include <stdlib.h>
 
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "gpioset.h"



void initGpio(int n)
{
    int lGpioFd;
    char lCache[100];
    lGpioFd =open("/sys/class/gpio/export", O_WRONLY);
    if (lGpioFd< 0)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("export open filed");
        #endif
    }
    int len = sprintf(lCache, "%d", n+338+78);
    write(lGpioFd, lCache, len+1);
    close(lGpioFd);
}   //create gpio file
 
void setGpioDirection(int n,char *direction)
{
    int lGpioFd;
    char path[100] = {0};
    char lCache[100]={};
    sprintf(path,"/sys/class/gpio/gpio%d/direction",n+338+78);
    lGpioFd =open(path,O_RDWR);

    int len = sprintf(lCache, "%s", direction);
    write(lGpioFd, lCache, len+1);
    close(lGpioFd);
}   //set gpio "in" or "out"
 
/*
*** 设置IO口电平
*** 在设置IO电平之前,需要调用 gpio_direction_set
*/
int gpio_level_set(const int pin,unsigned int flag)
{
    char gpio_buf[64] =
    {
        0
    };
    sprintf(gpio_buf, "/sys/class/gpio/gpio%d/value", pin+338+78);
    int gpio_fd = open(gpio_buf, O_WRONLY);

    if (gpio_fd < 0)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("gpio%d set value open failed \n", pin);
        #endif
        return -1;
    }

    if(flag==1)
    {
        if (write(gpio_fd, "1", 1) < 0)
        {

            close(gpio_fd);
            return -1;

        }
     }
    else
     {
        if (write(gpio_fd, "0", 1) < 0)
        {

            close(gpio_fd);
            return -1;

        }
    }

    close(gpio_fd);
    return 1;
}

void setGpioValue(int n, int value)
{
    char path[100]  = {0};
    sprintf(path, "/sys/class/gpio/gpio%d/value", n+338+78);
    FILE *fp = fopen(path, "w");
    if (fp == NULL)
        perror("value open filed");
    else
        fprintf(fp, "%d", value);
    fclose(fp);
}

int getGpioValue(int n)
{
    char path[64];
    char value_str[3];
    int fd;

    sprintf(path,"/sys/class/gpio/gpio%d/value",n+338+78);
    fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open gpio value for reading!");
        return -1;
    }
 
    if (read(fd, value_str, 3) < 0) {
        perror("Failed to read value!");
        return -1;
    }
 
    close(fd);
    return (atoi(value_str));
}   //get gpio(n)'s value


unsigned int getGpioValueMutiple(int n,int num)
{
    char path[64];
    char value_str[3];
    int fd;
    int i;
    unsigned int value=0;
    unsigned int value_temp;
    for(i=0;i<num;i++)
    {

        sprintf(path,"/sys/class/gpio/gpio%d/value",n+338+78+i);
        fd = open(path, O_RDONLY);
        if (fd < 0) {
            perror("Failed to open gpio value for reading!");
            return -1;
        }

        if (read(fd, value_str, 3) < 0) {
            perror("Failed to read value!");
            return -1;
        }
        value_temp=atoi(value_str);
        value|=((value_temp&0x1)<<i);
        close(fd);
     }
    //return (atoi(value_str));
    return value;

}   //get gpio(n)'s value

