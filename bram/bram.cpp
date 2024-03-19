#include"bram.h"
#include<stdio.h>
#include<stdlib.h>
#include <ctype.h>
#include <string.h>
#include <memory.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/mman.h>



int initBram(bramInfo *pBramInfo,unsigned int baseAddr,int memsize)
{
    pBramInfo->memsize = memsize;
    pBramInfo->max_size = MAX_MEM_LEN;
    pBramInfo->mapSaddr = NULL;
    pBramInfo->baseAddr = baseAddr;
    pBramInfo->flag=0;

    if(pBramInfo->flag == 0xAA)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("Bram has been mapped\n");
        #endif
        return -1;
    }
    if(pBramInfo->max_size<pBramInfo->memsize)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("memsize can't big than %x\n",pBramInfo->max_size);
        #endif
        return -1;
    }
    if(pBramInfo->baseAddr<=0)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("bram baseAddr is a illegal value m_bramInfo.baseAddr=%x\n",pBramInfo->baseAddr);
        #endif
        return -1;
    }
    pBramInfo->mmapfd = open("/dev/mem", O_RDWR | O_SYNC);
    if( pBramInfo->mmapfd < 0)
    {
        #ifdef IF_PRINTF_MONITOR
        printf("initBram():open /dev/mem failed ! \n");
        #endif
        return -1;
    }
    //printf("initBram():open /dev/mem success ! \n");
    pBramInfo->mapSaddr = mmap(NULL, memsize, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED ,
                               pBramInfo->mmapfd, pBramInfo->baseAddr);
    if(pBramInfo->mapSaddr == MAP_FAILED)
    {
        return -1;
    }
    pBramInfo->flag = 0xAA;
    return 1;
}

int readBram(bramInfo *ptrBramInfo,unsigned char data[],int offset,int len)
{

    if((ptrBramInfo->flag!=0xAA)||(offset+len>ptrBramInfo->max_size)
      ||(data == NULL)||(len<0)||(offset<0))
    {
        #ifdef IF_PRINTF_MONITOR
        printf("readBram failed!,m_bramInfo.flag=%x,offset+len=%x,data=%p,len=%d,offset=%d\n",
               ptrBramInfo->flag,offset+len,data,len,offset);
        #endif
        return -1;
    }
    unsigned char *tPtr = (unsigned char*)(ptrBramInfo->mapSaddr);
    memcpy(data,tPtr+offset,len);
    return len;
}
int writeBram(bramInfo *ptrBramInfo,unsigned char data[],int offset,int len)
{

    if((ptrBramInfo->flag!=0xAA)||(offset+len>ptrBramInfo->max_size)
      ||(data == NULL)||(len<0)||(offset<0))
    {
        #ifdef IF_PRINTF_MONITOR
        printf("readBram failed!,m_bramInfo.flag=%x,offset+len=%x,data=%p,len=%d,offset=%d\n",
               ptrBramInfo->flag,offset+len,data,len,offset);
        #endif
        return -1;
    }
    unsigned char *tPtr = (unsigned char*)(ptrBramInfo->mapSaddr);
    memcpy(tPtr+offset,data,len);
    return len;
}
